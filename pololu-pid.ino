/*
 Name:		pololu_pid.ino
 Created:	4/3/2022 4:00:33 PM
 Author:	Scott Scherzer

 Wheelspeed will be initialized to zero.
 After the head servo does a full sweep the data controller will decide what wall to follow

*/

#include <Servo.h>
#include <Pololu3piPlus32U4OLED.h>
#include <Pololu3piPlus32U4Motors.h>
#include <Pololu3piPlus32U4Buzzer.h>
#include <Pololu3piPlus32U4.h>
#include "UltrasonicController.h"
#include "PIDController.h"
#include "ServoData.h"
#include "DataController.h"

using namespace Pololu3piPlus32U4;

/* global data */

constexpr int POS_LEN = 7;
constexpr int PINGS_PER_ANGLE = 20;
constexpr float MAX_DISTANCE = 200.0f;
constexpr float US_MIN_DISTANCE = 2.0f;

Servo headServo;
Motors motors;

Ultrasonic us(false, 1L, 22, 21);
DataController data(false, 100L);

PID sidePID(false, 60L, 9.0f); // period a bit after ping
PID fwdPID(false, 100L); // period a bit after ping

/* These are only for data. Dont manipulate any hardware */
ServoData servoData(false, 100L, 20);

// timers
unsigned long servoTimer1 = 0L, servoTimer2 = 0L;
unsigned long usTimer1 = 0L, usTimer2 = 0L;
unsigned long motorTimer1 = 0L, motorTimer2 = 0L;

//motor data
const unsigned long MOTOR_PERIOD = 20L;
const int MIN_SPEED = 40;
const int MAX_SPEED = 100;
int leftSpeed = MIN_SPEED, rightSpeed = MIN_SPEED;

// target distances 
const float TGT_DISTANCES[POS_LEN] = { 82.18f, 82.31f, 30.0f, 15.0f, 30.0f, 100.0f, 80.0f };

// container for last average distance of each angle
float distances[POS_LEN] = { };

// pid containers
float left1Correction = 0.0f, left2Correction = 0.0f, fwdCorrection = 0.0f;

// the setup function runs once when you press reset or power the board
void setup() { 

  Serial.begin(9600); 
  Serial.println(""); //newline

  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);

  headServo.attach(servoData.PIN);
  headServo.write(90);

  delay(1000);

  servoData.ready = true;
  servoData.waiting = false;
  us.ready = false;
  data.ready = false;
}

// the loop function runs over and over again until power down or reset
// choose which routine to run based on time elapsed since start of cycle
void loop() {

  setServo();
  readUltrasonic();
  filterData();

  setMotorsSpeeds();
}

void setServo()
{
  servoTimer1 = millis();

  // poll servo
  if (servoData.ready)
  {
    //HACK
    data.resetRollingAvg();

    servoData.sweepHead();
    headServo.write(servoData.getAngle());

    servoData.ready = false;
    servoData.waiting = true;

    servoTimer2 = servoTimer1;
  }

  // delay to allow servo to finish its sweep, and delay before calling ping
  else if (servoTimer1 > (servoTimer2 + servoData.PERIOD) && servoData.waiting)
  {
    //servoData.debug();
    
    servoData.waiting = false;
    us.ready = true;
  }
}

void readUltrasonic()
{
  usTimer1 = millis();

  //send one ping, write to correct index
  if (usTimer1 > usTimer2 + us.PERIOD && us.ready)
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

      //us.debug();
    }

    //store last time ran
    usTimer2 = usTimer1;
  }
}

void filterData()
{
  // after all pings have sent, write the average to the array
  if (data.ready)
  {
    distances[servoData.getPosition()] = data.getAvgDistance();

    //after we store the avg clear the members
    data.resetRollingAvg();

    // side pid corrections
    sidePID.setCurrentError(distances[0], TGT_DISTANCES[0]);
    left1Correction = sidePID.calculatePID(sidePID.getCurrentError());

    // fwd pid correction
    fwdPID.setCurrentError(distances[4], TGT_DISTANCES[4]);
    fwdCorrection = fwdPID.calculatePID(fwdPID.getCurrentError());

    //HACK change to motors
    data.ready = false;
    servoData.ready = true;

    //data.debug();
    //sidePID.debug();
    //fwdPID.debug();
    //debugDistances();
  }
}

void setMotorsSpeeds()
{
  motorTimer1 = millis();

  if (motorTimer1 > motorTimer2 + MOTOR_PERIOD)
  {
    if (left1Correction < 0)
      motors.setSpeeds(MIN_SPEED, MIN_SPEED + 10);
    else
      motors.setSpeeds(MIN_SPEED + 10, MIN_SPEED);

    //too close, turn 
    if (fwdCorrection > 0)
      motors.setSpeeds(leftSpeed++ , rightSpeed = MIN_SPEED);

    motorTimer2 = motorTimer1;
  }
}

void debugDistances()
{
  Serial.print("dist  | ");
  Serial.print(distances[0]);
  Serial.print(" | ");
  Serial.print(distances[1]);
  Serial.print(" | ");
  Serial.print(distances[2]);
  Serial.print(" | ");
  Serial.print(distances[3]);
  Serial.print(" | ");
  Serial.print(distances[4]);
  Serial.print(" | ");
  Serial.print(distances[5]);
  Serial.print(" | ");
  Serial.print(distances[6]);
  Serial.println(" | ");
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
