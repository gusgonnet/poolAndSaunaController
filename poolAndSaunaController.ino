// The MIT License (MIT)
// Copyright (c) 2016 Gustavo Gonnet, Sergio Boyd
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software
// and associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
// is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies
// or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
// PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
// OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// github: https://github.com/gusgonnet/poolAndSaunaController
// hackster: https://www.hackster.io/gusgonnet/TBD ** TBD
//
// Free for personal use.
//
// Uses this library for the DS18b20 sensor: https://github.com/LukeUSMC/ds18b20-photon
// Datasheet: http://cdn.sparkfun.com/datasheets/Sensors/Temp/DS18B20.pdf
//

#include "Particle-OneWire.h"
#include "DS18B20.h"
#include "NCD4Relay.h"

#include "application.h"
#include "elapsedMillis.h"
#include "FiniteStateMachine.h"

#define APP_NAME "poolAndSaunaController"
String VERSION = "Version 0.04";

SYSTEM_MODE(AUTOMATIC);

/*******************************************************************************
 * changes in version 0.01:
       * Initial version
 * changes in version 0.02:
       * Logic is working now
 * changes in version 0.03:
       * Fixing unstable temperature samples
       * adding minimum time in state
       * adding setCalibration function
       * added turnOffAllRelays at boot time
 * changes in version 0.04:
       * Storing settings in EEPROM
*******************************************************************************/

// Argentina time zone GMT-3
const int TIME_ZONE = -3;

/*******************************************************************************
 declare FSM states with proper enter, update and exit functions
 we currently have the following states:
  - init: when the system boots up it starts in this state.
           After some time (ten seconds), the system transitions to the off state.
  - off:  the target temperature is being monitored.
           No relays are activated. Water pump is off
  - on :  the water pump is activated until the target temperature is reached.
           Relay for the water pump is on.

State changes:
Most likely: off -> on -> off -> on and so on
States at power up: init -> off

*******************************************************************************/
State initState = State(initEnterFunction, initUpdateFunction, initExitFunction);
State offState = State(offEnterFunction, offUpdateFunction, offExitFunction);
State onState = State(onEnterFunction, onUpdateFunction, onExitFunction);

//initialize state machine, start in state: init
FSM pumpStateMachine = FSM(initState);

// Sample FSM every now and then - 100 milliseconds means 10 times per second
#define QUICK_LOOP_INTERVAL 100
elapsedMillis quickLoopTimer;

// FSM states constants
#define STATE_INIT "Initializing"
#define STATE_OFF "Off"
#define STATE_ON "On"
String state = STATE_INIT;

// timers work on millis, so we adjust the value with this constant
#define MILLISECONDS_TO_MINUTES 60000
#define MILLISECONDS_TO_SECONDS 1000

// 10 seconds for the init cycle, so sensor samples get stabilized
// units: seconds
int initTimeout = 10;

// a state has to have a minimum duration time: this is now one minute
// units: minutes
int minimumTimeInState = 1;

/*******************************************************************************
 temperature sensor and variables
*******************************************************************************/
//Sets Pin D2 for Temp Sensor
DS18B20 ds18b20 = DS18B20(D2);

// Sample temperature sensor every 30 seconds
#define TEMPERATURE_SAMPLE_INTERVAL 30 * MILLISECONDS_TO_SECONDS
elapsedMillis temperatureSampleInterval;

//temperature related variables
#define INVALID -1
double temperatureCurrent = INVALID;
double temperatureTarget = 30.0;

//you can change this to your liking
// a smaller value will make your temperature more constant at the price of
//  starting the pump more often
// a larger value will reduce the number of times the pump turns on but will leave it on a longer time
double temperatureMargin = 0.25;

//sensor difference with real temperature (if none set to zero)
//use this variable to align measurements with your existing displays
double temperatureCalibration = 0;

// Celsius is the default unit, set this boolean to true if you want to use Fahrenheit
const bool useFahrenheit = false;

/*******************************************************************************
 relay variables
*******************************************************************************/
NCD4Relay relayController;

/*******************************************************************************
 structure for writing thresholds in eeprom
 https://docs.particle.io/reference/firmware/photon/#eeprom
*******************************************************************************/
//randomly chosen value here. The only thing that matters is that it's not 255
// since 255 is the default value for uninitialized eeprom
// value 137 will be used in version 0.4
#define EEPROM_VERSION 137
#define EEPROM_ADDRESS 0

struct EepromMemoryStructure
{
  uint8_t version = EEPROM_VERSION;
  double temperatureTarget;
  double temperatureCalibration;
};
EepromMemoryStructure eepromMemory;

/*******************************************************************************
 * Function Name  : setup
 * Description    : this function runs once at system boot
 *******************************************************************************/
void setup()
{

  // publish startup message with firmware version
  Particle.publish(APP_NAME, VERSION, PRIVATE);

  // declare cloud variables
  // https://docs.particle.io/reference/firmware/photon/#particle-variable-
  // Up to 20 cloud variables may be registered and each variable name is limited to a maximum of 12 characters.
  Particle.variable("temperature", temperatureCurrent);
  Particle.variable("target", temperatureTarget);
  Particle.variable("calibration", temperatureCalibration);

  // This variable informs the state of the system
  Particle.variable("state", state);

  // declare cloud functions
  // https://docs.particle.io/reference/firmware/photon/#particle-function-
  // Up to 15 cloud functions may be registered and each function name is limited to a maximum of 12 characters.
  Particle.function("setTarget", setTarget);
  Particle.function("setCalbrtion", setCalibration);

  Time.zone(TIME_ZONE);

  relayController.setAddress(0, 0, 0);

  // this is needed if you are flashing new versions while there is one or more relays activated
  // they will remain active if we don't turn them off
  relayController.turnOffAllRelays();

  //restore settings from eeprom, if there were any saved before
  readFromEeprom();
}

/*******************************************************************************
 * Function Name  : loop
 * Description    : this function runs continuously while the project is running
 *******************************************************************************/
void loop()
{
  quickLoop();
}

/*******************************************************************************
 * Function Name  : quickLoop
 * Description    : this function runs every 100 milliseconds (10 times a second)
                     to save power
 *******************************************************************************/
void quickLoop()
{

  // is time up? no, then come back later
  if (quickLoopTimer < QUICK_LOOP_INTERVAL)
  {
    return;
  }

  // time is up, reset timer
  quickLoopTimer = 0;

  // read the temperature sensor
  readTemperature();

  // update the FSM
  // the FSM is the heart of the program - all actions are defined by its states
  pumpStateMachine.update();
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 CONTROL FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 * Function Name  : setTarget
 * Description    : this function sets the target temperature
 * Parameters     : String parameter: the target temperature
                     Value has to be between 0 and 125. Anything else is ignored
 * Return         : 0 if success, -1 if fails
 *******************************************************************************/
int setTarget(String parameter)
{

  double localValue = atoi(parameter);

  if ((localValue >= 0) && (localValue <= 125))
  {
    temperatureTarget = localValue;
    Particle.publish(APP_NAME, "Setting temperature target to " + double2string(temperatureTarget), PRIVATE);
    saveSettingsInEeprom();
    return 0;
  }

  Particle.publish(APP_NAME, "Failed to set termperature target to " + parameter + " (0<=value<=125)", PRIVATE);
  return -1;
}

/*******************************************************************************
 * Function Name  : setCalibration
 * Description    : this function sets the calibration of the temperature
 * Parameters     : String parameter: the calibration value
                     Value has to be between -50 and 50. Anything else is ignored
 * Return         : 0 if success, -1 if fails
 *******************************************************************************/
int setCalibration(String parameter)
{

  double localValue = atoi(parameter);

  if ((localValue >= -50) && (localValue <= 50))
  {

    // rollback previous calibration adjustment
    temperatureCurrent = temperatureCurrent - temperatureCalibration;

    // store new calibration value
    temperatureCalibration = localValue;

    // apply new calibration adjustment
    temperatureCurrent = temperatureCurrent + temperatureCalibration;

    Particle.publish(APP_NAME, "Setting temperature calibration to " + double2string(temperatureCalibration), PRIVATE);
    saveSettingsInEeprom();
    return 0;
  }

  Particle.publish(APP_NAME, "Failed to set temperature calibration to " + parameter + " (-50<=value<=50)", PRIVATE);
  return -1;
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 SENSOR FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 * Function Name  : readTemperature
 * Description    : reads the temperature sensor at every TEMPERATURE_SAMPLE_INTERVAL
 * Return         : none
 *******************************************************************************/
void readTemperature()
{

  // is time up? no, then come back later
  if (temperatureSampleInterval < TEMPERATURE_SAMPLE_INTERVAL)
  {
    return;
  }

  //is time up, reset timer
  temperatureSampleInterval = 0;

  getTemp();

  Particle.publish(APP_NAME, "Temperature: " + double2string(temperatureCurrent), PRIVATE);
}

/*******************************************************************************
 * Function Name  : double2string
 * Description    : return the string representation of the double number
                     passed as parameter with 2 decimals
 * Return         : the string
 *******************************************************************************/
String double2string(double doubleNumber)
{
  String stringNumber = String(doubleNumber);

  //return only 2 decimals
  // Example: show 19.00 instead of 19.000000
  stringNumber = stringNumber.substring(0, stringNumber.length() - 4);

  return stringNumber;
}

void getTemp()
{

  int dsAttempts = 0;
  double temperatureLocal = INVALID;

  if (!ds18b20.search())
  {
    ds18b20.resetsearch();
    temperatureLocal = ds18b20.getTemperature();
    while (!ds18b20.crcCheck() && dsAttempts < 4)
    {
      dsAttempts++;
      if (dsAttempts == 3)
      {
        delay(1000);
      }
      ds18b20.resetsearch();
      temperatureLocal = ds18b20.getTemperature();
      continue;
    }
    dsAttempts = 0;

    if (useFahrenheit)
    {
      temperatureLocal = ds18b20.convertToFahrenheit(temperatureLocal);
    }

    // calibrate values
    temperatureLocal = temperatureLocal + temperatureCalibration;

    // if reading is valid, take it
    if ((temperatureLocal != INVALID) && (ds18b20.crcCheck()))
    {
      temperatureCurrent = temperatureLocal;
    }
  }
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 FINITE STATE MACHINE FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 * FSM state Name        : init 
 * Description           : when the system boots starts in this state to stabilize sensors.
                            After some time (ten seconds), the system transitions to the idle state.
* Status of the pump     : off
*******************************************************************************/
void initEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_INIT);
}
void initUpdateFunction()
{
  // is time up? no, then come back later
  if (pumpStateMachine.timeInCurrentState() < (initTimeout * MILLISECONDS_TO_SECONDS))
  {
    return;
  }

  pumpStateMachine.transitionTo(offState);
}
void initExitFunction()
{
}

/*******************************************************************************
 * FSM state Name        : off 
 * Description           : the system is checking the temperature. When it goes under a threshold, then
                            the On state is triggered.
 * Status of the pump    : off
*******************************************************************************/
void offEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_OFF);
}
void offUpdateFunction()
{

  // is minimum time up? no, then come back later
  if (pumpStateMachine.timeInCurrentState() < (minimumTimeInState * MILLISECONDS_TO_MINUTES))
  {
    return;
  }

  //if the temperature is lower than the threshold, transition to onState
  // temperature is equal to INVALID at boot time until a valid temperature is measured
  if ((temperatureCurrent != INVALID) && (temperatureCurrent <= temperatureTarget - temperatureMargin))
  {
    pumpStateMachine.transitionTo(onState);
  }
}
void offExitFunction()
{
}

/*******************************************************************************
 * FSM state Name        : On 
 * Description           : the pump will be turned on until the temperature reaches the target.
                            It then transitions to the off state.
 * Status of the pump    : on
*******************************************************************************/
void onEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_ON);

  turnOnPump();
}
void onUpdateFunction()
{

  // is minimum time up? no, then come back later
  if (pumpStateMachine.timeInCurrentState() < (minimumTimeInState * MILLISECONDS_TO_MINUTES))
  {
    return;
  }

  //if the temperature is higher than the threshold, transition to offState
  // temperature is equal to INVALID at boot time until a valid temperature is measured
  if ((temperatureCurrent != INVALID) && (temperatureCurrent >= temperatureTarget + temperatureMargin))
  {
    pumpStateMachine.transitionTo(offState);
  }
}
void onExitFunction()
{
  turnOffPump();
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 HELPER FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 * Function Name  : setState
 * Description    : sets the state of the system and publishes the change
 * Return         : none
 *******************************************************************************/
void setState(String newState)
{
  state = newState;
  Particle.publish(APP_NAME, "Entering " + newState + " state", PRIVATE);
}

/*******************************************************************************
 * Function Name  : turnOnPump
 * Description    : turns on the pump, closing the relay
 * Return         : none
 *******************************************************************************/
void turnOnPump()
{
  relayController.turnOnRelay(1);
  Particle.publish(APP_NAME, "Turning on pump", PRIVATE);
}

/*******************************************************************************
 * Function Name  : turnOffPump
 * Description    : turns off the pump, by opening the relay
 * Return         : none
 *******************************************************************************/
void turnOffPump()
{
  relayController.turnOffRelay(1);
  Particle.publish(APP_NAME, "Turning off pump", PRIVATE);
}

/*******************************************************************************/
/*******************************************************************************/
/*******************          EEPROM FUNCTIONS         *************************/
/********  https://docs.particle.io/reference/firmware/photon/#eeprom         **/
/********                                                                     **/
/********  wear and tear discussion:                                          **/
/********  https://community.particle.io/t/eeprom-flash-wear-and-tear/23738/5 **/
/*******************************************************************************/
/*******************************************************************************/

/*******************************************************************************
 * Function Name  : readFromEeprom
 * Description    : retrieves the settings from the EEPROM memory
 * Return         : none
 *******************************************************************************/
void readFromEeprom()
{

  EepromMemoryStructure myObj;
  EEPROM.get(EEPROM_ADDRESS, myObj);

  //verify this eeprom was written before
  // if version is 255 it means the eeprom was never written in the first place, hence the
  // data just read with the previous EEPROM.get() is invalid and we will ignore it
  if (myObj.version == EEPROM_VERSION)
  {

    temperatureTarget = myObj.temperatureTarget;
    temperatureCalibration = myObj.temperatureCalibration;

    Particle.publish(APP_NAME, "Read settings from EEPROM");
  }
}

/*******************************************************************************
 * Function Name  : saveSettingsInEeprom
 * Description    : This function saves interesting data to EEPROM
 * Return         : none
 *******************************************************************************/
void saveSettingsInEeprom()
{

  //store variables in the struct type that will be saved in the eeprom
  eepromMemory.version = EEPROM_VERSION;
  eepromMemory.temperatureTarget = temperatureTarget;
  eepromMemory.temperatureCalibration = temperatureCalibration;

  //then save
  EEPROM.put(EEPROM_ADDRESS, eepromMemory);

  Particle.publish(APP_NAME, "Stored settings on EEPROM");
}
