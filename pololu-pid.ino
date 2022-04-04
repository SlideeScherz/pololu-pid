/*
 Name:		pololu_pid.ino
 Created:	4/3/2022 4:00:33 PM
 Author:	Scott Scherzer

 Wheelspeed will be initialized to zero.
 After the head servo does a full sweep the data controller will decide what wall to follow

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

constexpr int POS_LEN = 7;
constexpr int PINGS_PER_ANGLE = 20;
constexpr float MAX_DISTANCE = 200.0f;
constexpr float US_MIN_DISTANCE = 2.0f;

Ultrasonic us(false, 1L, 22, 21);
DataController data(false, 100L);

PID sidePID(false, 60L); // period a bit after ping
PID fwdPID(false, 100L); // period a bit after ping

/* These are only for data. Dont manipulate any hardware */
ServoData servoData(false, 100L, 20);
WheelData wheels(false, 100L);

//scheduler timer. time at the start of the cycle. Updated when all routines finished
unsigned long t1 = 0L;

enum eWall { NONE, LEFT, RIGHT };

// target distances 
const float TGT_DISTANCES[POS_LEN] = { 80.0f, 100.0f, 30.0f, 15.0f, 30.0f, 100.0f, 80.0f };

// container for last average distance of each angle
float distances[POS_LEN] = { };

// the setup function runs once when you press reset or power the board
void setup() { 

  Serial.begin(9600); 
  Serial.println(""); //newline

  data.wallTarget = eWall::NONE;

  // set 45 and 135 to NULL to show they havent been seen yet
  distances[0] = NULL;
  distances[6] = NULL;

  servoData.ready = true;
  servoData.waiting = false;
  us.ready = false;
  data.ready = false;
}

// the loop function runs over and over again until power down or reset
// choose which routine to run based on time elapsed since start of cycle
void loop() {

  //get current time
  t1 = millis();

  servoData.debug();
  us.debug();
  data.debug();
 
  Serial.print("dist  | ");
  for (int i = 0; i < 7; i++)
  {
    Serial.print(distances[i]);
    Serial.print(" | ");
  }
  Serial.println("");

  // poll servo
  if (servoData.ready)
  {
    servoData.sweepHead();
    //headServo.write(angle);

    servoData.ready = false;
    servoData.waiting = true;

    servoData.t2 = t1;
  }

  // delay to allow servo to finish its sweep, and delay before calling ping
  else if (t1 > (servoData.t2 + servoData.PERIOD) && servoData.waiting)
  {
    servoData.waiting = false;
    us.ready = true;
    
  }

  //send one ping, write to correct index
  else if (t1 > us.t2 + us.PERIOD && us.ready)
  {

    //send ping
    us.setPingDistance();

    //fetch from US, and get a moving avg from data
    data.calcRollingAvg(us.getPingDistance(), us.getPingsSent());

    // mark this routine as complete
    if (us.getPingsSent() >= PINGS_PER_ANGLE)
    {
      
      us.setPingsSent(0);

      us.ready = false;
      data.ready = true;
    }

    //store last time ran
    us.t2 = t1;
  }

  // after all pings have sent, write the average to the array
  else if (data.ready)
  {
    distances[servoData.getPosition()] = data.getAvgDistance();

    //after we store the avg clear the members
    data.resetRollingAvg();

    // calculate the current error
    sidePID.setCurrentError(distances[servoData.getPosition()], TGT_DISTANCES[servoData.getPosition()]);

    // fwd pid correction
    //fwdCorrection = fwdPID.calculatePID(fwdPID.getCurrentError());

    // side pid corrections
    //sideCorrection = sidePID.calculatePID(sidePID.getCurrentError());

    data.ready = false;
  }
   
  // dont adjust wheels yet if we havent seen both sides
  else if (distances[0] == NULL || distances[6] == NULL)
  {
    servoData.ready = true;
    return;
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
