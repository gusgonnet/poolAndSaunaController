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

#include "application.h"
#include "elapsedMillis.h"
#include "FiniteStateMachine.h"

#define APP_NAME "poolAndSaunaController"
String VERSION = "Version 0.01";

/*******************************************************************************
 * changes in version 0.01:
       * Initial version
*******************************************************************************/

// Argentina time zone GMT-5
const int TIME_ZONE = -5;

/*******************************************************************************
 declare FSM states with proper enter, update and exit functions
 we currently have the following states:
  - init: when the system boots up it starts in this state.
           After some time (ten seconds), the system transitions to the idle state.
  - idle:  the target temperature is being monitored.
           No relays are activated.
  - on  :  the water pump is activated until the target temperature is reached.
           Relay for the water pump is on.

State changes:
Most likely: idle -> on -> idle -> on and so on
States at power up: init -> idle

*******************************************************************************/
State initState = State(initEnterFunction, initUpdateFunction, initExitFunction);
State idleState = State(idleEnterFunction, idleUpdateFunction, idleExitFunction);
State onState = State(onEnterFunction, onUpdateFunction, onExitFunction);

//initialize state machine, start in state: init
FSM valveStateMachine = FSM(initState);

// Sample FSM every now and then - 100 milliseconds means 10 times per second
#define QUICK_LOOP_INTERVAL 100
elapsedMillis quickLoopTimer;

// Report system status every now and then. This is for now 2 minutes
#define REPORT_STATUS_INTERVAL 120000
elapsedMillis reportStatusInterval;
String status = "";

// FSM states constants
#define STATE_INIT "Initializing"
#define STATE_IDLE "Idle"
#define STATE_ON "On"
String state = STATE_INIT;

// timers work on millis, so we adjust the value with this constant
#define MILLISECONDS_TO_MINUTES 60000
#define MILLISECONDS_TO_SECONDS 1000

// seconds for the init cycle, so sensor samples get stabilized
// let's try 10 seconds
// units: seconds
const int initTimeout = 10;

/*******************************************************************************
 IO mapping
*******************************************************************************/
// D5 : temperature sensor
int turnValveOnOutput = D3;
int turnValveOffOutput = D4;
int temperatureSensorInput = A0;

/*******************************************************************************
 moisture sensor and variables
*******************************************************************************/
// Sample moisture sensor every now and then (5 secs)
#define MOISTURE_SAMPLE_INTERVAL 5000
elapsedMillis moistureSampleInterval;

// how many samples to take and average
//  the higher the number the longer it takes, but measurement is smoother
const int NUMBER_OF_SAMPLES = 10;

// This is c1 in the spreadsheet
// c1: The system shall store the raw moisture value in a Particle cloud variable - READ only
// -1 means invalid
#define INVALID -1
int moistureCurrent = INVALID;

// This is c4 in the spreadsheet
// c4: The system shall store a moisture threshold in a Particle cloud variable - configurable by the user
int moistureThreshold = 1800;

/*******************************************************************************
 ubidots variables

 webhook definition:
  Event name: sunpick
  url: https://things.ubidots.com/api/v1.6/devices/{{ubi-dsl-vl}}/values/?token={{ubi-token}}
  Request type: POST
  Device: Any
  Advanced settings:
  Send custom data: JSON  
  and then enter:
    {
    "value": "{{ubi-value}}"
    }
  include default data: no
  enforce ssl: yes
*******************************************************************************/
// This value comes from ubidots
const String ubidotsToken = UBIDOTS_TOKEN;

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
  uint8_t mode;
  int moistureThreshold;
  int valveOnTime;
  int restTime;
  unsigned long valveOnToday;
  unsigned long onYesterday;
  unsigned long on2daysAgo;
  unsigned long on3daysAgo;
  unsigned long on4daysAgo;
  unsigned long on5daysAgo;
  unsigned long on6daysAgo;
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

  // declare and init pins
  pinMode(turnValveOnOutput, OUTPUT);
  pinMode(turnValveOffOutput, OUTPUT);
  pinMode(moistureSensorInput, INPUT);
  digitalWrite(turnValveOnOutput, LOW);
  digitalWrite(turnValveOffOutput, LOW);

  // declare cloud variables
  // https://docs.particle.io/reference/firmware/photon/#particle-variable-
  // Up to 20 cloud variables may be registered and each variable name is limited to a maximum of 12 characters.

  // This is c1 in our spreadsheet
  // c1: The system shall store the raw moisture value in a Particle cloud variable - READ only
  Particle.variable("moisture", moistureCurrent);

  // This is c4 in the spreadsheet
  // c4: The system shall store a moisture threshold in a Particle cloud variable - configurable by the user
  Particle.variable("threshold", moistureThreshold);

  // This is c5 in the spreadsheet
  // c5: The system shall store a maximum ON time for the valve in a Particle cloud variable - configurable by the user
  Particle.variable("valveOnTime", valveOnTime);

  // This is c5.1 in the spreadsheet
  // c5.1: The system shall store a minimum interval between ON time - configurable by the user
  // AKA: number of minutes to leave the system in rest state
  // This gives enough time for the terrain to absorb the water and the moisture level to raise accordingly
  Particle.variable("restTime", restTime);

  // This is c6 in the spreadsheet
  // c6: The system shall store a pulse duration for working the valve in a Particle cloud variable - configurable by the user
  Particle.variable("valvePulse", valvePulse);

  // This variable informs the status of the system
  Particle.variable("status", status);

  // This variable informs the status of the valve
  Particle.variable("valveStatus", valveStatus);

  // This variable informs the number of minutes the valve was on today (so far)
  Particle.variable("valveOnToday", valveOnTotal[0]);

  // This variable informs the number of minutes the valve was on yesterday
  Particle.variable("onYesterday", valveOnTotal[1]);

  // This variable informs the number of minutes the valve was on 2 days ago
  Particle.variable("on2daysAgo", valveOnTotal[2]);

  // This variable informs the number of minutes the valve was on 3 days ago
  Particle.variable("on3daysAgo", valveOnTotal[3]);

  // This variable informs the number of minutes the valve was on 4 days ago
  Particle.variable("on4daysAgo", valveOnTotal[4]);

  // This variable informs the number of minutes the valve was on 5 days ago
  Particle.variable("on5daysAgo", valveOnTotal[5]);

  // This variable informs the number of minutes the valve was on 6 days ago
  Particle.variable("on6daysAgo", valveOnTotal[6]);

  // This variable informs the number of minutes the valve was on this week so far
  Particle.variable("valveOnWeek", valveOnWeek);

  // declare cloud functions
  // https://docs.particle.io/reference/firmware/photon/#particle-function-
  // Up to 15 cloud functions may be registered and each function name is limited to a maximum of 12 characters.
  Particle.function("turnOnOff", turnOnOff);
  Particle.function("setThreshold", setThreshold);
  Particle.function("setValveOnTi", setValveOnTime);
  Particle.function("setRestTime", setRestTime);

  Time.zone(TIME_ZONE);

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
  slowLoop();
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

  // read the moisture sensor
  readMoisture();

  // update the FSM
  // the FSM is the heart of the program - all actions are defined by its states
  valveStateMachine.update();

  slowLoop();

  reportStatus();
}

/*******************************************************************************
 * Function Name  : slowLoop
 * Description    : this loop gets executed few times PER HOUR (every 20 minutes in this case)
 * Return         : none
 *******************************************************************************/
void slowLoop()
{
  // is time up? no, then come back later
  if (slowLoopTimer < SLOW_LOOP_INTERVAL * MILLISECONDS_TO_MINUTES)
  {
    return;
  }

  // time is up, reset timer
  slowLoopTimer = 0;

  updateLongTermData();

  saveSettingsInEeprom();

  publishToUbidots("valveOnToday", String(valveOnTotal[0]));
  delay(1000);
  publishToUbidots("onYesterday", String(valveOnTotal[1]));
  delay(1000);
  publishToUbidots("valveOnWeek", String(valveOnWeek));
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 CONTROL FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 * Function Name  : turnOffAuto
 * Description    : this function turns the system on/off
 * Parameters     : String parameter: if equal to MODE_ON ("On") the system turns on
                     if equal to MODE_OFF ("Off") the system turns off
                     Anything else is ignored
 * Return         : 0 if success, -1 if fails
 *******************************************************************************/
int turnOnOff(String parameter)
{
  if ((parameter == MODE_ON) || (parameter == "on") || (parameter == "ON"))
  {
    mode = MODE_ON;
    Particle.publish(APP_NAME, "Setting mode to " + parameter, PRIVATE);

    // setting the status interval here ensures that the next status report will be done soon enough
    //  otherwise the status of the system shows invalid information for up to REPORT_STATUS_INTERVAL
    reportStatusInterval = REPORT_STATUS_INTERVAL - 2000;

    return 0;
  }

  if ((parameter == MODE_OFF) || (parameter == "off") || (parameter == "OFF"))
  {
    mode = MODE_OFF;
    Particle.publish(APP_NAME, "Setting mode to " + parameter, PRIVATE);

    // setting the status interval here ensures that the next status report will be done soon enough
    //  otherwise the status of the system shows invalid information for up to REPORT_STATUS_INTERVAL
    reportStatusInterval = REPORT_STATUS_INTERVAL - 2000;

    return 0;
  }

  Particle.publish(APP_NAME, "Failed to set mode to " + parameter, PRIVATE);
  return -1;
}

/*******************************************************************************
 * Function Name  : setThreshold
 * Description    : this function sets the threshold
 * Parameters     : String parameter: the moisture threshold
                     Value has to be between 0 and 4095. Anything else is ignored
 * Return         : 0 if success, -1 if fails
 *******************************************************************************/
int setThreshold(String parameter)
{

  int localValue = atoi(parameter);

  if ((localValue >= 0) && (localValue <= 4095))
  {
    moistureThreshold = localValue;
    Particle.publish(APP_NAME, "Setting threshold to " + parameter, PRIVATE);

    // setting the status interval here ensures that the next status report will be done soon enough
    //  otherwise the status of the system shows invalid information for up to REPORT_STATUS_INTERVAL
    reportStatusInterval = REPORT_STATUS_INTERVAL - 2000;

    return 0;
  }

  Particle.publish(APP_NAME, "Failed to set threshold to " + parameter + " (0<=value<=4095)", PRIVATE);
  return -1;
}

/*******************************************************************************
 * Function Name  : setValveOnTime
 * Description    : this function sets the valve on duration
 * Parameters     : String parameter: the valve on duration in minutes
                     Value has to be between 1 and 1440. Anything else is ignored
 * Return         : 0 if success, -1 if fails
 *******************************************************************************/
int setValveOnTime(String parameter)
{

  int localValue = atoi(parameter);

  if ((localValue >= 1) && (localValue <= 1440))
  {
    valveOnTime = localValue;
    Particle.publish(APP_NAME, "Setting valveOnTime to " + parameter, PRIVATE);

    // setting the status interval here ensures that the next status report will be done soon enough
    //  otherwise the status of the system shows invalid information for up to REPORT_STATUS_INTERVAL
    reportStatusInterval = REPORT_STATUS_INTERVAL - 2000;

    return 0;
  }

  Particle.publish(APP_NAME, "Failed to set valveOnTime to " + parameter + " (1<=value<=1440)", PRIVATE);
  return -1;
}

/*******************************************************************************
 * Function Name  : setRestTime
 * Description    : this function sets the rest duration
 * Parameters     : String parameter: the rest duration in minutes
                     Value has to be between 1 and 1440. Anything else is ignored
 * Return         : 0 if success, -1 if fails
 *******************************************************************************/
int setRestTime(String parameter)
{

  int localValue = atoi(parameter);

  if ((localValue >= 1) && (localValue <= 1440))
  {
    restTime = localValue;
    Particle.publish(APP_NAME, "Setting restTime to " + parameter, PRIVATE);

    // setting the status interval here ensures that the next status report will be done soon enough
    //  otherwise the status of the system shows invalid information for up to REPORT_STATUS_INTERVAL
    reportStatusInterval = REPORT_STATUS_INTERVAL - 2000;

    return 0;
  }

  Particle.publish(APP_NAME, "Failed to set restTime to " + parameter + " (1<=value<=1440)", PRIVATE);
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
 * Function Name  : readMoisture
 * Description    : reads the moisture sensor at every MOISTURE_SAMPLE_INTERVAL
 * Return         : none
 *******************************************************************************/
void readMoisture()
{

  // is time up? no, then come back later
  if (moistureSampleInterval < MOISTURE_SAMPLE_INTERVAL)
  {
    return;
  }

  //is time up, reset timer
  moistureSampleInterval = 0;

  //let's make an average of the measured temperature
  // by taking N samples
  uint8_t i;
  int moistureAverage;
  int moistureSamples[NUMBER_OF_SAMPLES];

  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMBER_OF_SAMPLES; i++)
  {
    moistureSamples[i] = analogRead(moistureSensorInput);
    delay(10);
  }

  // average all the samples out
  moistureAverage = 0;
  for (i = 0; i < NUMBER_OF_SAMPLES; i++)
  {
    moistureAverage += moistureSamples[i];
  }
  moistureAverage /= NUMBER_OF_SAMPLES;

  // sample acquired and averaged
  moistureCurrent = moistureAverage;
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
* Status of the valve    : closed
*******************************************************************************/
void initEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_INIT);
}
void initUpdateFunction()
{
  // is time up? no, then come back later
  if (valveStateMachine.timeInCurrentState() < (initTimeout * MILLISECONDS_TO_SECONDS))
  {
    return;
  }

  // decide to which state to transition based on the current mode of the system
  if (mode == MODE_ON)
  {
    valveStateMachine.transitionTo(idleState);
  }
  else
  {
    valveStateMachine.transitionTo(offState);
  }
}
void initExitFunction()
{
}

/*******************************************************************************
 * FSM state Name        : idle 
 * Description           : the system is checking the moisture. When it goes under a threshold, then
                            the On state is triggered.
* Status of the valve    : closed
*******************************************************************************/
void idleEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_IDLE);
}
void idleUpdateFunction()
{

  // if the user sets the mode to off, transition to the FSM off state
  if (mode == MODE_OFF)
  {
    valveStateMachine.transitionTo(offState);
  }

  //if the moisture is lower than the threshold, transition to valveOnState
  // moisture is equal to INVALID at boot time
  if ((moistureCurrent != INVALID) && (moistureCurrent <= moistureThreshold))
  {
    valveStateMachine.transitionTo(valveOnState);
  }
}
void idleExitFunction()
{
}

/*******************************************************************************
 * FSM state Name        : On 
 * Description           : the valve will be open for a fixed period of time.
                            Once this period finishes, the system transitions to the rest state.
* Status of the valve    : open
*******************************************************************************/
void onEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_ON);

  openValve();
}
void onUpdateFunction()
{

  // if the user sets the mode to off, transition to the FSM off state
  if (mode == MODE_OFF)
  {
    valveStateMachine.transitionTo(offState);
  }

  // is time up? no, then come back later
  if (valveStateMachine.timeInCurrentState() < (valveOnTime * MILLISECONDS_TO_MINUTES))
  {
    return;
  }

  // update the valve on counter
  // NOTE: THIS HAS TO BE DONE BEFORE CALLING the transitionTo function!!!!
  // since calling that function resets the timer :(
  valveOnTotal[0] = valveOnTotal[0] + valveStateMachine.timeInCurrentState() / MILLISECONDS_TO_MINUTES;

  updateWeeklyTotal();

  // transition to rest once the ON period expires (valveOnTime)
  valveStateMachine.transitionTo(restState);
}
void onExitFunction()
{
  closeValve();
}

/*******************************************************************************
 * FSM state Name        : rest 
 * Description           : the system waits for a period of time until allowing the next valve cycle to take place.
                            Once this period finishes, the system transitions to the idle state.
* Status of the valve    : closed
*******************************************************************************/
void restEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_REST);
}
void restUpdateFunction()
{

  // if the user sets the mode to off, transition to the FSM off state
  if (mode == MODE_OFF)
  {
    valveStateMachine.transitionTo(offState);
  }

  // is time up? no, then come back later
  if (valveStateMachine.timeInCurrentState() < (restTime * MILLISECONDS_TO_MINUTES))
  {
    return;
  }

  valveStateMachine.transitionTo(idleState);
}
void restExitFunction()
{
}

/*******************************************************************************
 * FSM state Name        : off 
 * Description           : the system is off. The valve will not be activated. Only the user can switch
                            from this state to the idle state.
* Status of the valve    : closed
*******************************************************************************/
void offEnterFunction()
{
  // set the new state and publish the change
  setState(STATE_OFF);
}
void offUpdateFunction()
{
  // if the user sets the mode to on, transition to the FSM on state
  if (mode == MODE_ON)
  {
    valveStateMachine.transitionTo(idleState);
  }
}
void offExitFunction()
{
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
  Particle.publish(APP_NAME, "Entering " + newState + " state", PRIVATE);
  state = newState;
}

/*******************************************************************************
 * Function Name  : reportStatus
 * Description    : logs the status of the system
 * Return         : none
 *******************************************************************************/
void reportStatus()
{
  // is time up? no, then come back later
  if (reportStatusInterval < REPORT_STATUS_INTERVAL)
  {
    return;
  }

  // time is up, reset timer
  reportStatusInterval = 0;

  // update status variable
  status = "State: " + state + ", for the last " + String(valveStateMachine.timeInCurrentState() / MILLISECONDS_TO_MINUTES) + " minutes";

  // publish status
  Particle.publish(APP_NAME, status, PRIVATE);
  delay(1000);
  Particle.publish(APP_NAME, "Moisture: " + String(moistureCurrent) + ", Threshold: " + moistureThreshold, PRIVATE);
  delay(1000);

  publishData();
}


/*******************************************************************************
 * Function Name  : updateLongTermData
 * Description    : updates the long term data: the time the valve has been on for today and the last few days
                     checks if it's midnight, and if it is, it updates the long term data
 * Notes          : WARNING: THIS FUNCTION CAN TAKE A LONG TIME TO EXECUTE (few seconds...)
 * Return         : none
 *******************************************************************************/
void updateLongTermData()
{
  // updates happen at midnight
  if (Time.hour() != 0)
  {
    longTermDataUpdatedForToday = false;
    return;
  }

  // did we update already for today?
  if (longTermDataUpdatedForToday)
  {
    return;
  }

  // flag that we updated the data for today
  longTermDataUpdatedForToday = true;

  // REMEMBER THAT:
  // valveOnTotal[0] will contain today's total so far
  // valveOnTotal[1] will contain yesterday's total
  // valveOnTotal[2] will contain 2 days ago total
  // valveOnTotal[3] will contain 3 days ago total
  // valveOnTotal[4] will contain 4 days ago total
  // valveOnTotal[5] will contain 5 days ago total
  // valveOnTotal[6] will contain 6 days ago total
  // units: minutes
  // unsigned long valveOnTotal[7];

  // we have to copy todays data to yesterdays, and so on so forth
  valveOnTotal[6] = valveOnTotal[5];
  valveOnTotal[5] = valveOnTotal[4];
  valveOnTotal[4] = valveOnTotal[3];
  valveOnTotal[3] = valveOnTotal[2];
  valveOnTotal[2] = valveOnTotal[1];
  valveOnTotal[1] = valveOnTotal[0];

  // now reset todays total
  valveOnTotal[0] = 0;

  // update total of the week
  updateWeeklyTotal();
}

/*******************************************************************************
 * Function Name  : updateWeeklyTotal
 * Description    : updates the weekly grand total of the valve 
 * Return         : none
 *******************************************************************************/
void updateWeeklyTotal()
{
  valveOnWeek = valveOnTotal[6] + valveOnTotal[5] + valveOnTotal[4] + valveOnTotal[3] + valveOnTotal[2] + valveOnTotal[1] + valveOnTotal[0];
}

/*******************************************************************************
 * Function Name  : openValve
 * Description    : opens the valve by pulsing the turnValveOnOutput output
 * Return         : none
 *******************************************************************************/
void openValve()
{
  Particle.publish(APP_NAME, "Opening valve", PRIVATE);
  digitalWrite(turnValveOnOutput, HIGH);
  delay(valvePulse);
  digitalWrite(turnValveOnOutput, LOW);

  // update valve status variable
  valveStatus = VALVE_ON;
}

/*******************************************************************************
 * Function Name  : closeValve
 * Description    : opens the valve by pulsing the turnValveOffOutput output
 * Return         : none
 *******************************************************************************/
void closeValve()
{
  Particle.publish(APP_NAME, "Closing valve", PRIVATE);
  digitalWrite(turnValveOffOutput, HIGH);
  delay(valvePulse);
  digitalWrite(turnValveOffOutput, LOW);

  // update valve status variable
  valveStatus = VALVE_OFF;
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 UBIDOTS FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 * Function Name  : publishData
 * Description    : sends the value of the data we want to ubidots
 * Notes          : WARNING: THIS FUNCTION CAN TAKE A LONG TIME TO EXECUTE (few seconds...)
 * Return         : none
 *******************************************************************************/
void publishData()
{

  if (moistureCurrent != INVALID)
  {
    publishToUbidots("moisture", String(moistureCurrent));
  }

  delay(1000);
  publishToUbidots("threshold", String(moistureThreshold));
  delay(1000);
  String valveInt = "0";
  if (valveStatus == VALVE_ON)
  {
    valveInt = "1";
  }
  publishToUbidots("valveStatus", valveInt);
}

/*******************************************************************************
 * Function Name  : publishToUbidots
 * Description    : sends the value of the parameter to ubidots
 * Parameters     : String name: name of the value
                    String value: value to store in ubidots
 * Return         : none
 *******************************************************************************/
void publishToUbidots(String name, String value)
{
  Particle.publish("sunpick", "{\"ubi-dsl-vl\":\"" + Particle.deviceID() + "/" + name + "\", \"ubi-token\":\"" + ubidotsToken + "\", \"ubi-value\":\"" + value + "\"}", 60, PRIVATE);
}

/*******************************************************************************/
/*******************************************************************************/
/*******************          EEPROM FUNCTIONS         *************************/
/********  https://docs.particle.io/reference/firmware/photon/#eeprom         **/
/********                                                                     **/
/********  wear and tear discussion:                                          **/
/********  https://community.particle.io/t/eeprom-flash-wear-and-tear/23738/5 **/
/**                                                                           **/
/** If we write 11 int and long int variables (~44 bytes) to EEPROM every 20  **/
/** minutes, the EEPROM wil wear out in aproximately 172 years, so I think    **/
/** we are good. Math:                                                        **/
/** we can write 200 million bytes to eeprom before it wears out (read link)  **/
/** 200 million/40bytes/3timesPerHour/24hours/365days == around 172 years     **/
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

    mode = convertIntToMode(myObj.mode);
    moistureThreshold = myObj.moistureThreshold;
    valveOnTime = myObj.valveOnTime;
    restTime = myObj.restTime;
    valveOnTotal[0] = myObj.valveOnToday;
    valveOnTotal[1] = myObj.onYesterday;
    valveOnTotal[2] = myObj.on2daysAgo;
    valveOnTotal[3] = myObj.on3daysAgo;
    valveOnTotal[4] = myObj.on4daysAgo;
    valveOnTotal[5] = myObj.on5daysAgo;
    valveOnTotal[6] = myObj.on6daysAgo;

    // update total of the week
    updateWeeklyTotal();

    Particle.publish(APP_NAME, "Read settings from EEPROM");
  }
}

/*******************************************************************************
 * Function Name  : saveSettingsInEeprom
 * Description    : in this function we wait a bit to give the user time
                    to adjust the right value for them and in this way we try not
                    to save in EEPROM at every little change.
                    Remember that each eeprom writing cycle is a precious and finite resource
 * Return         : none
 *******************************************************************************/
void saveSettingsInEeprom()
{

  //store thresholds in the struct type that will be saved in the eeprom
  eepromMemory.version = EEPROM_VERSION;
  eepromMemory.mode = convertModeToInt(mode);
  eepromMemory.moistureThreshold = moistureThreshold;
  eepromMemory.valveOnTime = valveOnTime;
  eepromMemory.restTime = restTime;
  eepromMemory.valveOnToday = valveOnTotal[0];
  eepromMemory.onYesterday = valveOnTotal[1];
  eepromMemory.on2daysAgo = valveOnTotal[2];
  eepromMemory.on3daysAgo = valveOnTotal[3];
  eepromMemory.on4daysAgo = valveOnTotal[4];
  eepromMemory.on5daysAgo = valveOnTotal[5];
  eepromMemory.on6daysAgo = valveOnTotal[6];

  //then save
  EEPROM.put(EEPROM_ADDRESS, eepromMemory);

  Particle.publish(APP_NAME, "Stored settings on EEPROM");
}

/*******************************************************************************
 * Function Name  : convertIntToMode
 * Description    : converts the int mode (saved in the eeprom) into the String mode (in RAM)
 * Return         : String
 *******************************************************************************/
String convertIntToMode(uint8_t mode)
{
  if (mode == 1)
  {
    return MODE_ON;
  }

  //in all other cases
  return MODE_OFF;
}

/*******************************************************************************
 * Function Name  : convertModeToInt
 * Description    : converts the String mode (in RAM) into the int mode (to be saved in the eeprom)
 * Return         : String
 *******************************************************************************/
uint8_t convertModeToInt(String mode)
{
  if (mode == MODE_ON)
  {
    return 1;
  }

  //in all other cases
  return 0;
}
