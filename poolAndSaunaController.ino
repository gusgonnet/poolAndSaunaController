// Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
// This is a human-readable summary of (and not a substitute for) the license.
// Disclaimer
// 
// You are free to:
// Share — copy and redistribute the material in any medium or format
// Adapt — remix, transform, and build upon the material
// The licensor cannot revoke these freedoms as long as you follow the license terms.
// 
// Under the following terms:
// Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
// NonCommercial — You may not use the material for commercial purposes.
// ShareAlike — If you remix, transform, or build upon the material, you must distribute your contributions under the same license as the original.
// No additional restrictions — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.
// 
// Notices:
// You do not have to comply with the license for elements of the material in the public domain or where your use is permitted by an applicable exception or limitation.
// No warranties are given. The license may not give you all of the permissions necessary for your intended use. For example, other rights such as publicity, privacy, or moral rights may limit how you use the material.
//
// github: https://github.com/gusgonnet/poolAndSaunaController
// hackster: https://www.hackster.io/gusgonnet/pool-and-sauna-controller-b24a9a
//
// Free for personal use.
//
// https://creativecommons.org/licenses/by-nc-sa/4.0/
//
// Uses this library for the DS18b20 sensor: https://github.com/LukeUSMC/ds18b20-photon
// Datasheet: http://cdn.sparkfun.com/datasheets/Sensors/Temp/DS18B20.pdf
//

/*
NOTE: To draw the FSM image, copy paste this uml code below in http://www.plantuml.com/plantuml/uml

@startuml
skinparam backgroundColor LightYellow
skinparam state {
  BackgroundColor LightBlue
  BorderColor Gray
  FontName Impact
}


[*] --> initState

note left of initState : The system boots\nin this state

initState: 10 seconds
initState -down--> offState: if system was\npreviously off
initState -down--> idleState: if system was\npreviously on


offState: system off
offState -left-> idleState: setOnOff("on")

idleState: heating off
idleState -down-> onState: temperature\n< target
idleState -right-> offState: setOnOff("off")

onState: heating on
onState -up-> idleState: temperature\n> target
onState -up-> offState: setOnOff("off")
@enduml
*/

#include "Particle-OneWire.h"
#include "DS18B20.h"
#include "NCD4Relay.h"
#include "PietteTech_DHT.h"
#include "elapsedMillis.h"
#include "FiniteStateMachine.h"

#define APP_NAME "poolAndSaunaController"
String VERSION = "Version 0.09";

SYSTEM_MODE(AUTOMATIC);

/*******************************************************************************
 * changes in version 0.01:
       * Initial version
 * changes in version 0.02:
       * Logic is working now
       * pool ds18b20 sensor on D2
 * changes in version 0.03:
       * Fixing unstable temperature samples
       * adding minimum time in state
       * adding setCalibration function
       * added turnOffAllRelays at boot time
 * changes in version 0.04:
       * Storing settings in EEPROM
 * changes in version 0.05:
       * duplicating FSM and others to control pool and sauna
       * pool ds18b20 sensor on D3
 * changes in version 0.06:
       * adding system on/off
 * changes in version 0.07:
       * addding user control for relays 3 and 4 (via a cloud function)
       * adding third ds18b20 sensor for sensing ambient temperature on D4
 * changes in version 0.08:
       * upgrading control and adding function to get status of relays 3 and 4
 * changes in version 0.09:
       * adding DHT22 sensor for sensing ambient temperature and humidity on D5
*******************************************************************************/

// Argentina time zone GMT-3
const int TIME_ZONE = -3;

/*******************************************************************************
 declare FSM states with proper enter, update and exit functions
 we currently have the following states:
  - init: when the system boots up it starts in this state.
           After some time (ten seconds), the system transitions to the off or idle state.
  - off:  the system is off until the user sets it to on
           No relays are activated. Water pump is off
  - idle:  the target temperature is being monitored.
           No relays are activated. Water pump is off
  - on :  the water pump is activated until the target temperature is reached.
           Relay for the water pump is on.

State changes:
Most likely: off -> idle -> on -> idle -> on and so on
States at power up: init -> off if previous state was off
                    init -> idle if previous state was on or idle

*******************************************************************************/
State initState = State(initEnterFunction, initUpdateFunction, initExitFunction);
State offState = State(offEnterFunction, offUpdateFunction, offExitFunction);
State idleState = State(idleEnterFunction, idleUpdateFunction, idleExitFunction);
State onState = State(onEnterFunction, onUpdateFunction, onExitFunction);

//initialize state machine for the pool, start in state: init
FSM stateMachine1 = FSM(initState);

// Sample FSM every now and then - 100 milliseconds means 10 times per second
#define QUICK_LOOP_INTERVAL 100
elapsedMillis quickLoopTimer;

// FSM states constants
#define STATE_INIT "Initializing"
#define STATE_OFF "Off"
#define STATE_IDLE "Idle"
#define STATE_ON "On"
String state = STATE_INIT;
String lastState = "";

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

//sensor difference with real temperature (if none set to zero)
//use this variable to align measurements with your existing displays
double temperatureCalibration = 0;

//you can change this to your liking
// a smaller value will make your temperature more constant at the price of
//  starting the pump more often
// a larger value will reduce the number of times the pump turns on but will leave it on a longer time
double temperatureMargin = 0.25;

// Celsius is the default unit, set this boolean to true if you want to use Fahrenheit
const bool useFahrenheit = false;

/*******************************************************************************
 declare second FSM, with second sensor for the sauna
*******************************************************************************/
State initState2 = State(initEnterFunction2, initUpdateFunction2, initExitFunction2);
State offState2 = State(offEnterFunction2, offUpdateFunction2, offExitFunction2);
State idleState2 = State(idleEnterFunction2, idleUpdateFunction2, idleExitFunction2);
State onState2 = State(onEnterFunction2, onUpdateFunction2, onExitFunction2);
//initialize state machine for the sauna, start in state: init
FSM stateMachine2 = FSM(initState2);
//Sets Pin D3 for Temp Sensor2
DS18B20 ds18b20_2 = DS18B20(D3);
double temperatureCurrent2 = INVALID;
double temperatureTarget2 = 37.0;
double temperatureCalibration2 = 0;
String state2 = STATE_INIT;
String lastState2 = "";

/*******************************************************************************
 temperature sensor and variables for ambient sensing
*******************************************************************************/
//Sets Pin D4 for Ambient Temp Sensor
DS18B20 ds18b20_3 = DS18B20(D4);
double temperatureCurrent3 = INVALID;

/*******************************************************************************
 DHT sensor for ambient sensing
*******************************************************************************/
#define DHTTYPE  DHT22                // Sensor type DHT11/21/22/AM2301/AM2302
#define DHTPIN   5                    // Digital pin for communications
void dht_wrapper(); // must be declared before the lib initialization
PietteTech_DHT DHT(DHTPIN, DHTTYPE, dht_wrapper);
bool bDHTstarted;       // flag to indicate we started acquisition
double temperatureCurrent4 = INVALID;
double humidityCurrent4 = INVALID;
// This wrapper is in charge of calling the DHT sensor lib
void dht_wrapper() { DHT.isrCallback(); }

/*******************************************************************************
 relay variables
*******************************************************************************/
NCD4Relay relayController;

// timers for switching off the relays
#define MILLISECONDS_TO_MINUTES 60000
elapsedMillis timerOnRelay1;
elapsedMillis timerOnRelay2;
elapsedMillis timerOnRelay3;
elapsedMillis timerOnRelay4;
int turnOffRelay1AfterTime = 0;
int turnOffRelay2AfterTime = 0;
int turnOffRelay3AfterTime = 0;
int turnOffRelay4AfterTime = 0;

/*******************************************************************************
 structure for writing thresholds in eeprom
 https://docs.particle.io/reference/firmware/photon/#eeprom
*******************************************************************************/
//randomly chosen value here. The only thing that matters is that it's not 255
// since 255 is the default value for uninitialized eeprom
// value 137 will be used in version 0.4
// value 138 will be used in version 0.5
// value 139 will be used in version 0.6
#define EEPROM_VERSION 139
#define EEPROM_ADDRESS 0

struct EepromMemoryStructure
{
  uint8_t version = EEPROM_VERSION;
  double temperatureTarget;
  double temperatureCalibration;
  uint8_t lastState;
  double temperatureTarget2;
  double temperatureCalibration2;
  uint8_t lastState2;
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
  Particle.function("setOnOff", setOnOff);
  Particle.function("setTarget", setTarget);
  Particle.function("setCalbrtion", setCalibration);

  /*******************************************************************************
   cloud variables and functions for the second FSM, with second sensor
  *******************************************************************************/
  Particle.variable("temperature2", temperatureCurrent2);
  Particle.variable("target2", temperatureTarget2);
  Particle.variable("calibration2", temperatureCalibration2);
  Particle.variable("state2", state2);
  Particle.function("setOnOff2", setOnOff2);
  Particle.function("setTarget2", setTarget2);
  Particle.function("setCalbrtn2", setCalibration2);

  /*******************************************************************************
   cloud variables and functions for the ambient temperature
  *******************************************************************************/
  Particle.variable("tempAmbient", temperatureCurrent3);

  Particle.variable("tempAmbieDHT", temperatureCurrent4);
  Particle.variable("humiAmbieDHT", humidityCurrent4);

  Time.zone(TIME_ZONE);

  // this function allows the user to control relays 3 and 4 (from IFTTT for example)
  Particle.function("controlRelay", triggerRelay);
  // this function allows the user to query the status of relays 3 and 4 (from IFTTT for example)
  Particle.function("relayStatus", relayStatus);
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

  // read the temperature sensors
  readTemperature();

  // update the FSMs
  // the FSM is the heart of the program - all actions are defined by its states
  stateMachine1.update();
  stateMachine2.update();

  // check if there is any relay to turn off automatically
  turnOffRelay();
  
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 CONTROL FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 * Function Name  : setOnOff
 * Description    : this function sets the system on or off
 * Parameters     : String parameter: on/ON/On or off/OFF/Off
 * Return         : 0 if success, -1 if fails
 *******************************************************************************/
int setOnOff(String parameter)
{

  if (parameter.equalsIgnoreCase("on"))
  {
    // transition to on only if the system is off
    if (state == STATE_OFF) {
      stateMachine1.transitionTo(idleState);
    }
    return 0;
  }
  if (parameter.equalsIgnoreCase("off"))
  {
    // transition to off only if the system is on
    if ( (state == STATE_IDLE) || (state == STATE_ON) ) {
      stateMachine1.transitionTo(offState);
    }
    return 0;
  }

  Particle.publish(APP_NAME, "Failed to set system FSM1 to " + parameter, PRIVATE);
  return -1;
}

/*******************************************************************************
 * Function Name  : setOnOff2
 * Description    : this function sets the system on or off for FSM2
 * Parameters     : String parameter: on/ON/On or off/OFF/Off
 * Return         : 0 if success, -1 if fails
 *******************************************************************************/
int setOnOff2(String parameter)
{

  if (parameter.equalsIgnoreCase("on"))
  {
    // transition to on only if the system is off
    if (state2 == STATE_OFF) {
      stateMachine2.transitionTo(idleState2);
    }
    return 0;
  }
  if (parameter.equalsIgnoreCase("off"))
  {
    // transition to off only if the system is on
    if ( (state2 == STATE_IDLE) || (state2 == STATE_ON) ) {
      stateMachine2.transitionTo(offState2);
    }
    return 0;
  }

  Particle.publish(APP_NAME, "Failed to set system FSM2 to " + parameter, PRIVATE);
  return -1;
}

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

  Particle.publish(APP_NAME, "Failed to set temperature target to " + parameter + " (0<=value<=125)", PRIVATE);
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
 * Function Name  : setTarget2
 * Description    : this function sets the target temperature for the sauna
 * Parameters     : String parameter: the target temperature
                     Value has to be between 0 and 125. Anything else is ignored
 * Return         : 0 if success, -1 if fails
 *******************************************************************************/
int setTarget2(String parameter)
{

  double localValue = atoi(parameter);

  if ((localValue >= 0) && (localValue <= 125))
  {
    temperatureTarget2 = localValue;
    Particle.publish(APP_NAME, "Setting temperature2 target to " + double2string(temperatureTarget2), PRIVATE);
    saveSettingsInEeprom();
    return 0;
  }

  Particle.publish(APP_NAME, "Failed to set temperature2 target to " + parameter + " (0<=value<=125)", PRIVATE);
  return -1;
}

/*******************************************************************************
 * Function Name  : setCalibration2
 * Description    : this function sets the calibration of the temperature2
 * Parameters     : String parameter: the calibration value
                     Value has to be between -50 and 50. Anything else is ignored
 * Return         : 0 if success, -1 if fails
 *******************************************************************************/
int setCalibration2(String parameter)
{

  double localValue = atoi(parameter);

  if ((localValue >= -50) && (localValue <= 50))
  {

    // rollback previous calibration adjustment
    temperatureCurrent2 = temperatureCurrent2 - temperatureCalibration2;

    // store new calibration value
    temperatureCalibration2 = localValue;

    // apply new calibration adjustment
    temperatureCurrent2 = temperatureCurrent2 + temperatureCalibration2;

    Particle.publish(APP_NAME, "Setting temperature2 calibration to " + double2string(temperatureCalibration2), PRIVATE);
    saveSettingsInEeprom();
    return 0;
  }

  Particle.publish(APP_NAME, "Failed to set temperature2 calibration to " + parameter + " (-50<=value<=50)", PRIVATE);
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
 * Description    : reads the temperature sensor in D2, D3 and D4 at every TEMPERATURE_SAMPLE_INTERVAL
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
  getTemp2();
  getTempAmb();
  getTempAmbDHT();

  Particle.publish(APP_NAME, "Pool: " + double2string(temperatureCurrent) \
    + ", Sauna: " + double2string(temperatureCurrent2) \
    + ", TAmb: " + double2string(temperatureCurrent3) \
    + ", TAmbDHT: " + double2string(temperatureCurrent4) \
    + ", HAmbDHT: " + double2string(humidityCurrent4) \
    , PRIVATE);
}

/*******************************************************************************
 * Function Name  : getTemp
 * Description    : reads the DS18B20 sensor
 * Return         : nothing
 *******************************************************************************/
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
 * Function Name  : getTemp2
 * Description    : reads the second DS18B20 sensor
 * Return         : nothing
 *******************************************************************************/
void getTemp2()
{

  int dsAttempts = 0;
  double temperatureLocal = INVALID;

  if (!ds18b20_2.search())
  {
    ds18b20_2.resetsearch();
    temperatureLocal = ds18b20_2.getTemperature();
    while (!ds18b20_2.crcCheck() && dsAttempts < 4)
    {
      dsAttempts++;
      if (dsAttempts == 3)
      {
        delay(1000);
      }
      ds18b20_2.resetsearch();
      temperatureLocal = ds18b20_2.getTemperature();
      continue;
    }
    dsAttempts = 0;

    if (useFahrenheit)
    {
      temperatureLocal = ds18b20_2.convertToFahrenheit(temperatureLocal);
    }

    // calibrate values
    temperatureLocal = temperatureLocal + temperatureCalibration2;

    // if reading is valid, take it
    if ((temperatureLocal != INVALID) && (ds18b20_2.crcCheck()))
    {
      temperatureCurrent2 = temperatureLocal;
    }
  }
}

/*******************************************************************************
 * Function Name  : getTempAmb
 * Description    : reads the third DS18B20 sensor (ambient)
 * Return         : nothing
 *******************************************************************************/
void getTempAmb()
{

  int dsAttempts = 0;
  double temperatureLocal = INVALID;

  if (!ds18b20_3.search())
  {
    ds18b20_3.resetsearch();
    temperatureLocal = ds18b20_3.getTemperature();
    while (!ds18b20_3.crcCheck() && dsAttempts < 4)
    {
      dsAttempts++;
      if (dsAttempts == 3)
      {
        delay(1000);
      }
      ds18b20_3.resetsearch();
      temperatureLocal = ds18b20_3.getTemperature();
      continue;
    }
    dsAttempts = 0;

    if (useFahrenheit)
    {
      temperatureLocal = ds18b20_3.convertToFahrenheit(temperatureLocal);
    }

    // if reading is valid, take it
    if ((temperatureLocal != INVALID) && (ds18b20_3.crcCheck()))
    {
      temperatureCurrent3 = temperatureLocal;
    }
  }
}

/*******************************************************************************
 * Function Name  : getTempAmbDHT
 * Description    : reads the temperature of the DHT22 sensor
 * Return         : none
 *******************************************************************************/
void getTempAmbDHT() {

  // start the sample
  if (!bDHTstarted) {
    DHT.acquireAndWait(5);
    bDHTstarted = true;
  }

  //still acquiring sample? go away and come back later
  if (DHT.acquiring()) {
    return;
  }

  //I observed my dht22 measuring below 0 from time to time, so let's discard that sample
  if ( DHT.getCelsius() < 0 ) {
    //reset the sample flag so we can take another
    bDHTstarted = false;
    return;
  }

    if (useFahrenheit)
    {
      temperatureCurrent4 = DHT.getFahrenheit();
    }
    else
    {
      temperatureCurrent4 = DHT.getCelsius();
    }

  humidityCurrent4 = DHT.getHumidity();

  //reset the sample flag so we can take another
  bDHTstarted = false;

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
                            After some time (ten seconds), the system transitions to the idle state or off state,
                             depending on previous system state
* Status of the pump     : off
*******************************************************************************/
void initEnterFunction()
{
  // set the new state and publish the change, do not set lastState since we need the value from the eeprom
  setState(STATE_INIT, false);
}
void initUpdateFunction()
{
  // is time up? no, then come back later
  if (stateMachine1.timeInCurrentState() < (initTimeout * MILLISECONDS_TO_SECONDS))
  {
    return;
  }

  if (lastState==STATE_IDLE)
  {
    stateMachine1.transitionTo(idleState);
  }
  else
  {
    stateMachine1.transitionTo(offState);
  }
}
void initExitFunction()
{
}

/*******************************************************************************
 * FSM state Name        : off 
 * Description           : the system is off until the user sets it to on
 * Status of the pump    : off
*******************************************************************************/
void offEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_OFF, true);
}
void offUpdateFunction()
{
}
void offExitFunction()
{
}

/*******************************************************************************
 * FSM state Name        : idle 
 * Description           : the system is checking the temperature. When it goes under a threshold, then
                            the On state is triggered.
 * Status of the pump    : off
*******************************************************************************/
void idleEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_IDLE, true);
}
void idleUpdateFunction()
{

  // is minimum time up? no, then come back later
  if (stateMachine1.timeInCurrentState() < (minimumTimeInState * MILLISECONDS_TO_MINUTES))
  {
    return;
  }

  //if the temperature is lower than the target, transition to onState
  // temperature is equal to INVALID at boot time until a valid temperature is measured
  if ((temperatureCurrent != INVALID) && (temperatureCurrent <= temperatureTarget - temperatureMargin))
  {
    stateMachine1.transitionTo(onState);
  }
}
void idleExitFunction()
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
  setState(STATE_ON, true);

  turnOnRelay(1);
}
void onUpdateFunction()
{

  // is minimum time up? no, then come back later
  if (stateMachine1.timeInCurrentState() < (minimumTimeInState * MILLISECONDS_TO_MINUTES))
  {
    return;
  }

  //if the temperature is higher than the target, transition to offState
  // temperature is equal to INVALID at boot time until a valid temperature is measured
  if ((temperatureCurrent != INVALID) && (temperatureCurrent >= temperatureTarget + temperatureMargin))
  {
    stateMachine1.transitionTo(idleState);
  }
}
void onExitFunction()
{
  turnOffRelay(1);
}

/*******************************************************************************
 * FSM state Name        : init2 
 * Description           : when the system boots starts in this state to stabilize sensors.
                            After some time (ten seconds), the system transitions to the idle state or off state,
                             depending on previous system state
* Status of the pump     : off
*******************************************************************************/
void initEnterFunction2()
{
  // set the new state and publish the change
  setState2(STATE_INIT, false);
}
void initUpdateFunction2()
{
  // is time up? no, then come back later
  if (stateMachine2.timeInCurrentState() < (initTimeout * MILLISECONDS_TO_SECONDS))
  {
    return;
  }

  if (lastState2==STATE_IDLE)
  {
    stateMachine2.transitionTo(idleState2);
  }
  else
  {
    stateMachine2.transitionTo(offState2);
  }
}
void initExitFunction2()
{
}

/*******************************************************************************
 * FSM state Name        : off2
 * Description           : the system is off until the user sets it to on
 * Status of the pump    : off
*******************************************************************************/
void offEnterFunction2()
{
  // set the new state and publish the change
  setState2(STATE_OFF, true);
}
void offUpdateFunction2()
{
}
void offExitFunction2()
{
}

/*******************************************************************************
 * FSM state Name        : idle2 
 * Description           : the system is checking the temperature. When it goes under a threshold, then
                            the On state is triggered.
 * Status of the pump    : off
*******************************************************************************/
void idleEnterFunction2()
{
  // set the new state and publish the change
  setState2(STATE_IDLE, true);
}
void idleUpdateFunction2()
{

  // is minimum time up? no, then come back later
  if (stateMachine2.timeInCurrentState() < (minimumTimeInState * MILLISECONDS_TO_MINUTES))
  {
    return;
  }

  //if the temperature2 is lower than the target2, transition to onState2
  // temperature is equal to INVALID at boot time until a valid temperature is measured
  if ((temperatureCurrent2 != INVALID) && (temperatureCurrent2 <= temperatureTarget2 - temperatureMargin))
  {
    stateMachine2.transitionTo(onState2);
  }
}
void idleExitFunction2()
{
}

/*******************************************************************************
 * FSM state Name        : On2 
 * Description           : the pump will be turned on until the temperature reaches the target.
                            It then transitions to the off state.
 * Status of the pump    : on
*******************************************************************************/
void onEnterFunction2()
{
  // set the new state and publish the change
  setState2(STATE_ON, true);

  turnOnRelay(2);
}
void onUpdateFunction2()
{

  // is minimum time up? no, then come back later
  if (stateMachine2.timeInCurrentState() < (minimumTimeInState * MILLISECONDS_TO_MINUTES))
  {
    return;
  }

  //if the temperature2 is higher than the target2, transition to offState2
  // temperature is equal to INVALID at boot time until a valid temperature is measured
  if ((temperatureCurrent2 != INVALID) && (temperatureCurrent2 >= temperatureTarget2 + temperatureMargin))
  {
    stateMachine2.transitionTo(idleState2);
  }
}
void onExitFunction2()
{
  turnOffRelay(2);
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 HELPER FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

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

/*******************************************************************************
 * Function Name  : setState
 * Description    : sets the state of the system and publishes the change
 * Return         : none
 *******************************************************************************/
void setState(String newState, bool setLastState)
{
  state = newState;
  if (setLastState) {
    lastState = newState;
  }

  Particle.publish(APP_NAME, "FSM1 entering " + newState + " state", PRIVATE);
  saveSettingsInEeprom();
}

/*******************************************************************************
 * Function Name  : setState2
 * Description    : sets the state of the second FSM and publishes the change
 * Return         : none
 *******************************************************************************/
void setState2(String newState, bool setLastState)
{
  state2 = newState;
  if (setLastState) {
    lastState2 = newState;
  }
  Particle.publish(APP_NAME, "FSM2 entering " + newState + " state", PRIVATE);
  saveSettingsInEeprom();
}

/*******************************************************************************
 * Function Name  : turnOnRelay
 * Description    : turns on the specified relay
 * Return         : none
 *******************************************************************************/
void turnOnRelay(int relay)
{
  relayController.turnOnRelay(relay);
  Particle.publish(APP_NAME, "Turning on relay " + String(relay), PRIVATE);
}

/*******************************************************************************
 * Function Name  : turnOffRelay
 * Description    : turns off the specified relay
 * Return         : none
 *******************************************************************************/
void turnOffRelay(int relay)
{
  relayController.turnOffRelay(relay);
  Particle.publish(APP_NAME, "Turning off relay " + String(relay), PRIVATE);
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
    lastState = convertIntToLastState(myObj.lastState);
    temperatureTarget2 = myObj.temperatureTarget2;
    temperatureCalibration2 = myObj.temperatureCalibration2;
    lastState2 = convertIntToLastState(myObj.lastState2);

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
  eepromMemory.lastState = convertLastStateToInt(lastState);
  eepromMemory.temperatureTarget2 = temperatureTarget2;
  eepromMemory.temperatureCalibration2 = temperatureCalibration2;
  eepromMemory.lastState2 = convertLastStateToInt(lastState2);

  //then save
  EEPROM.put(EEPROM_ADDRESS, eepromMemory);

  Particle.publish(APP_NAME, "Stored settings on EEPROM");
}

/*******************************************************************************
 * Function Name  : convertIntToLastState
 * Description    : converts the int lastState (saved in the eeprom) into the String mode (in RAM)
                    The logic here is that when the system was powered off, if it was IDLE or ON, 
                    we want it to transition to IDLE (and then automatically transition to ON if needed).
 * Return         : String
 *******************************************************************************/
String convertIntToLastState(uint8_t lastSt)
{

  // we only care to store the system being on (idleState or onState) or off (offState)
  // so if it was on/idle, we want to set the system to idle (and eventually it will turn on when the temp requires it)
  if (lastSt == 1)
  {
    return STATE_IDLE;
  }
  
  //in all other cases
  return STATE_OFF;
}

/*******************************************************************************
 * Function Name  : convertLastStateToInt
 * Description    : converts the String lastState (in RAM) into the int mode (to be saved in the eeprom)
                    The logic here is that when the system was powered off, if it was IDLE or ON, 
                    we want it to transition to IDLE (and then automatically transition to ON if needed).
 * Return         : String
 *******************************************************************************/
uint8_t convertLastStateToInt(String lastSt)
{

  // we only care to store the system being on (idleState or onState) or off (offState)
  if ( (lastSt == STATE_IDLE) || (lastSt == STATE_ON) )
  {
    return 1;
  }

  //in all other cases
  return 0;
}

/*******************************************************************************
 * Function Name  : relayStatus
 * Description    : this function returns 1 if the relay is on, 0 if the relay is off
                    and -1 if there is an error
 *******************************************************************************/
int relayStatus(String relay)
{
    int relayNumber = relay.substring(0, 1).toInt();
    if ((relayNumber >= 1) && (relayNumber <= 4))
    {
        return relayController.readRelayStatus(relayNumber);
    }

    return -1;
}

/*******************************************************************************
 * Function Name  : triggerRelay
 * Description    : commands accepted:
                        - "3on", "1off" to set on/off one relay at a time
                        - "34on", "12off", "13on", "13off" to set on/off more than one relay at a time
                        - "34on5", "1on3", "13on40" to turn on relay(s) for a number of minutes
                          example: "34on5" turns relays 3 and 4 on for 5 minutes
                        - 3toggle, 34toggle, 4momentary are also supported
 * Returns        : returns 1 if the command was accepted, 0 if not
 *******************************************************************************/
int triggerRelay(String command)
{

    // Relay Specific Command
    // this supports "34on", "12off", "13on", "13off" for example
    // this supports "34on5", "1on3", "13on40" to turn on relay(s) for a number of minutes
    String tempCommand = command.toLowerCase();
    // this var will store the number of minutes for a relay to be on
    int timeOn = 0;
    int relayNumber1 = 0;
    int relayNumber2 = 0;
    int relayNumber3 = 0;
    int relayNumber4 = 0;

    //parse the first relay number
    if (firstCharIsNumber1to4(tempCommand))
    {
        relayNumber1 = tempCommand.substring(0, 1).toInt();
        Serial.print("relayNumber1: ");
        Serial.println(relayNumber1);
        //then remove the first
        tempCommand = tempCommand.substring(1);
    }

    //check if first char is a digit
    if (firstCharIsNumber1to4(tempCommand))
    {
        //parse the second relay number
        relayNumber2 = tempCommand.substring(0, 1).toInt();
        Serial.print("relayNumber2: ");
        Serial.println(relayNumber2);
        //then remove the char
        tempCommand = tempCommand.substring(1);
    }

    //check if first char is a digit
    if (firstCharIsNumber1to4(tempCommand))
    {
        //parse the third relay number
        relayNumber3 = tempCommand.substring(0, 1).toInt();
        Serial.print("relayNumber3: ");
        Serial.println(relayNumber3);
        //then remove the char
        tempCommand = tempCommand.substring(1);
    }

    //check if first char is a digit
    if (firstCharIsNumber1to4(tempCommand))
    {
        //parse the fourth relay number
        relayNumber4 = tempCommand.substring(0, 1).toInt();
        Serial.print("relayNumber4: ");
        Serial.println(relayNumber4);
        //then remove the char
        tempCommand = tempCommand.substring(1);
    }

    //check if next chars are equal to on, if so, check if there was a specific ON time sent
    // it would be after the on
    // example: 34on50 for turning both relays 3 and 4 on for 50 minutes
    // when the program reaches this point, 34 have already been parsed
    // so here we would end up with on50
    if (tempCommand.startsWith("on"))
    {
        //parse the digits after the on command
        timeOn = tempCommand.substring(2).toInt();
        Serial.print("timeOn: ");
        Serial.println(timeOn);
        // set the command to be on, so the digits that came after are removed
        tempCommand = "on";
    }

    Serial.print("tempCommand:");
    Serial.print(tempCommand);
    Serial.println(".");

    int relayNumbers[4] = {relayNumber1, relayNumber2, relayNumber3, relayNumber4};

    int returnValue = 0;
    int relayNumber;
    int i;

    // explore all 4 relays
    for (i = 1; i <= 4; i++)
    {
        // analize this particular relay
        relayNumber = relayNumbers[i - 1];

        // allow only control of relays 3 and 4
        // since relays 1 and 2 are used by the pool and sauna system
        if (relayNumber > 2)
        {

            Particle.publish("command/relay/minutes", tempCommand + "/" + String(relayNumber) + "/" + String(timeOn), 60, PRIVATE);

            if (tempCommand.equalsIgnoreCase("on"))
            {
                Serial.println("Turning on relay");
                relayController.turnOnRelay(relayNumber);

                // if timeOn is not zero, then turn the relay off after timeOn minutes
                if (timeOn != 0)
                {
                    turnOnRelayForSomeMinutes(relayNumber, timeOn);
                }

                Serial.println("returning");
                returnValue = 1;
            }
            if (tempCommand.equalsIgnoreCase("off"))
            {
                relayController.turnOffRelay(relayNumber);
                returnValue = 1;
            }
            if (tempCommand.equalsIgnoreCase("toggle"))
            {
                relayController.toggleRelay(relayNumber);
                returnValue = 1;
            }
            if (tempCommand.equalsIgnoreCase("momentary"))
            {
                relayController.turnOnRelay(relayNumber);
                delay(300);
                relayController.turnOffRelay(relayNumber);
                returnValue = 1;
            }
        }
    }

    return returnValue;
}

/*******************************************************************************
 * Function Name  : firstCharIsNumber1to4
 * Description    : returns true if the parameter starts with a number between 1 and 4
 *******************************************************************************/
bool firstCharIsNumber1to4(String string)
{
    if (string.startsWith("1"))
    {
        return true;
    }
    if (string.startsWith("2"))
    {
        return true;
    }
    if (string.startsWith("3"))
    {
        return true;
    }
    if (string.startsWith("4"))
    {
        return true;
    }
    // not a 1~4 digit
    return false;
}

/*******************************************************************************
 * Function Name  : turnOnRelayForSomeMinutes
 * Description    : turn a relay on for a user defined time in minutes
 *******************************************************************************/
void turnOnRelayForSomeMinutes(int relay, int timeOn)
{

 switch(relay) {
     case 1:
      relayController.turnOnRelay(relay);
      timerOnRelay1 = 0;
      turnOffRelay1AfterTime = timeOn*MILLISECONDS_TO_MINUTES;
     break;
     case 2:
      relayController.turnOnRelay(relay);
      timerOnRelay2 = 0;
      turnOffRelay2AfterTime = timeOn*MILLISECONDS_TO_MINUTES;
     break;
     case 3:
      relayController.turnOnRelay(relay);
      timerOnRelay3 = 0;
      turnOffRelay3AfterTime = timeOn*MILLISECONDS_TO_MINUTES;
     break;
     case 4:
      relayController.turnOnRelay(relay);
      timerOnRelay4 = 0;
      turnOffRelay4AfterTime = timeOn*MILLISECONDS_TO_MINUTES;
     break;
 }


}

/*******************************************************************************
 * Function Name  : turnOffRelay
 * Description    : this function checks if there is a relay to turn off automatically
 *******************************************************************************/
void turnOffRelay()
{

  // is time up for relay1?
  if ((turnOffRelay1AfterTime!=0) && (timerOnRelay1 > turnOffRelay1AfterTime))
  {
      turnOffRelay1AfterTime = 0;
      relayController.turnOffRelay(1);
  }

  // is time up for relay2?
  if ((turnOffRelay2AfterTime!=0) && (timerOnRelay2 > turnOffRelay2AfterTime))
  {
      turnOffRelay2AfterTime = 0;
      relayController.turnOffRelay(2);
  }

  // is time up for relay3?
  if ((turnOffRelay3AfterTime!=0) && (timerOnRelay3 > turnOffRelay3AfterTime))
  {
      turnOffRelay3AfterTime = 0;
      relayController.turnOffRelay(3);
  }

  // is time up for relay4?
  if ((turnOffRelay4AfterTime!=0) && (timerOnRelay4 > turnOffRelay4AfterTime))
  {
      turnOffRelay4AfterTime = 0;
      relayController.turnOffRelay(4);
  }

}