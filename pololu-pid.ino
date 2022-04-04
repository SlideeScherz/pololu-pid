/*
 Name:		pololu_pid.ino
 Created:	4/3/2022 4:00:33 PM
 Author:	Scott Scherzer
*/

#include <Pololu3piPlus32U4OLED.h>
#include <Pololu3piPlus32U4Motors.h>
#include <Pololu3piPlus32U4Buzzer.h>
#include <Pololu3piPlus32U4.h>
#include "UltrasonicController.h"
#include "PIDController.h"
#include "ServoData.h"
#include "DataController.h"
#include "WheelData.h"

using namespace Pololu3piPlus32U4;

/* global data */

extern constexpr int POS_LEN = 7;
extern constexpr float MAX_DISTANCE = 200.0f;
extern constexpr float US_MIN_DISTANCE = 2.0f;
extern constexpr int PINGS_PER_ANGLE = 20;

Ultrasonic us(true, 1L, 22, 21);
DataController data(false, 100L);

PID sidePID(false, 60L); // period a bit after ping
PID fwdPID(false, 100L); // period a bit after ping

/* These are only for data. Dont manipulate any hardware */
ServoData servoData(false, 20L, 20);
WheelData wheels(false, 100L);

enum eStatus { WAITING, READY, DONE };

//scheduler timer. time at the start of the cycle. Updated when all routines finished
unsigned long s_t1 = 0L;

/* environment data */

//wall constants
enum eWall { NONE, LEFT, CENTER, RIGHT};

// target distances in respect to the angles in servo.headPositions[]
const float TGT_DISTANCES[POS_LEN] = { 80.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f , 0.0f };

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600); 
  Serial.println(); //newline

  us.setStatus(eStatus::WAITING);
  us.setPingsSent(0);

  checkBatteries();
}

// the loop function runs over and over again until power down or reset
void loop() {

  scheduler();

}

/*
 * Chose which routine to run based on time elapsed since start of cycle
 * @returns void. Scheduler will call the correct method
 */
void scheduler()
{
  //get current time
  s_t1 = millis();

  //send one ping, write to correct index
  //If servo is moving, it will set ultrasonic status to waiting
  if (s_t1 > us.t2 + us.PERIOD && us.getStatus() == eStatus::READY)
  {
    //send ping
    us.setPingDistance();

    //fetch from US, and get a moving avg from data
    data.calcRollingAvg(us.getPingDistance(), us.getPingsSent());

    //check if complete
    if (us.getPingsSent() >= PINGS_PER_ANGLE)
    {
      us.setPingsSent(0);
      us.setStatus(eStatus::DONE);
      data.setStatus(eStatus::READY);
    }

    //store last time ran
    us.t2 = s_t1;

    if (us.bDebug) us.debug();
  }

  //after all pings have sent, check and set status
  else if (s_t1 > data.t2 + data.PERIOD && data.getStatus() == eStatus::READY)
  {
    //push to allDistances Arr and reset
    data.allDistancesWrite(data.getAvgDistance(), servoData.getPosition());
    data.resetRollingAvg();

    data.setStatus(eStatus::DONE);
  }

  //PID correction
  else if (s_t1 > sidePID.PERIOD);
  {

  }

  
}

/*
 * set the LEDS to on or off.
 * @param (color)State 0 (off) or 1 (on)
 * @returns void. Sets the pololu LED pins
 */
extern void setLEDs(int yellowState, int greenState, int redState) {
  ledYellow(yellowState);
  ledGreen(greenState);
  ledRed(redState);
}

//Abort if battery low
void checkBatteries()
{
  Serial.print("[Batteries] ");
  Serial.println(readBatteryMillivolts());
}
