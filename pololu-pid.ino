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

constexpr int POS_LEN = 7;
constexpr int PINGS_PER_ANGLE = 20;
constexpr float MAX_DISTANCE = 200.0f;
constexpr float US_MIN_DISTANCE = 2.0f;

Ultrasonic us(true, 1L, 22, 21);
DataController data(false, 100L);

PID sidePID(false, 60L); // period a bit after ping
PID fwdPID(false, 100L); // period a bit after ping

/* These are only for data. Dont manipulate any hardware */
ServoData servoData(false, 100L, 20);
WheelData wheels(false, 100L);

//scheduler timer. time at the start of the cycle. Updated when all routines finished
unsigned long t1 = 0L;

enum eWall { NONE, LEFT, RIGHT };

// target distances in respect to the angles in servo.headPositions[]
const float TGT_DISTANCES[POS_LEN] = { 80.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f , 0.0f };

// container for last average distance of each angle
float distances[POS_LEN] = { };

// container for the error of each angle
float errors[POS_LEN] = { };

// container for the PID outputs
float corrections[POS_LEN] = { };

// the setup function runs once when you press reset or power the board
void setup() { 

  Serial.begin(9600); 
  Serial.println(); //newline

  data.wallTarget = eWall::NONE;

  servoData.ready = true;
  us.ready = false;
}

// the loop function runs over and over again until power down or reset
// choose which routine to run based on time elapsed since start of cycle
void loop() {

  //get current time
  t1 = millis();

  // poll servo
  if (servoData.ready)
  {
    servoData.sweepHead();
    //headServo.write(angle);

    servoData.t2 = t1;

    //allow servo to finish its sweep
    if (t1 > servoData.t2 + servoData.PERIOD)
    {
      servoData.ready = false;
      us.ready = true;
    }
  }

  //send one ping, write to correct index
  //if servo is moving, it will set ultrasonic status to waiting
  else if (t1 > us.t2 + us.PERIOD && us.ready)
  {
    //send ping
    us.setPingDistance();

    //fetch from US, and get a moving avg from data
    data.calcRollingAvg(us.getPingDistance(), us.getPingsSent());

    // mark this routine as complete
    if (us.getPingsSent() >= PINGS_PER_ANGLE)
    {
      data.resetRollingAvg();
      us.setPingsSent(0);
      us.ready = false;
      data.ready = true;
    }

    //store last time ran
    us.t2 = t1;

    if (us.bDebug) us.debug();
  }

  // after all pings have sent, write the average to the array
  else if (data.ready)
  {
    distances[servoData.getPosition()] = data.getAvgDistance();

    // pick a wall to follow
    // hack bad way to see if a sweep was fully complete
    if (data.wallTarget == eWall::NONE && distances[0] != 0.0f && distances[6] != 0.0f)
    {
      data.wallTarget = (distances[0] <= distances[6]) ? eWall::LEFT : eWall::RIGHT;
    }


    // calculate the current error
    sidePID.setCurrentError(distances[servoData.getPosition()], TGT_DISTANCES[servoData.getPosition()]);

    // store in error array 
    errors[servoData.getPosition()] = sidePID.getCurrentError();


    // left side pid correction
    if (servoData.getPosition() < 2)
    {
      corrections[0] = sidePID.calculatePID(errors[0]);
      corrections[1] = sidePID.calculatePID(errors[1]);
    }

    //fwd pid correction
    else if (servoData.getPosition() < 5)
    {
      corrections[2] = fwdPID.calculatePID(errors[2]);
      corrections[3] = fwdPID.calculatePID(errors[3]);
      corrections[4] = fwdPID.calculatePID(errors[4]);
    }
    
    // right side correction
    else if (servoData.getPosition() <= 6)
    corrections[5] = sidePID.calculatePID(errors[5]);
    corrections[6] = sidePID.calculatePID(errors[6]);

    data.ready = false;
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
