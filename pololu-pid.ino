/*
 Name:		pololu_pid.ino
 Created:	4/3/2022 4:00:33 PM
 Author:	Scott Scherzer

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

/* These are only for data. Dont manipulate any hardware */
ServoData servoData(false, 80L, 20);
DataController data(false, 100L);
PID sidePID(false, 50L); // period a bit after ping
PID fwdPID(false, 50L); // period a bit after ping

// if the servo is moving, make the US wait
bool servoMoving = false;

// timers
unsigned long servoTimer1 = 0L, servoTimer2 = 0L;
unsigned long usTimer1 = 0L, usTimer2 = 0L;
unsigned long motorTimer1 = 0L, motorTimer2 = 0L;
unsigned long pidTimer1 = 0L, pidTimer2 = 0L;

//motor data
const unsigned long MOTOR_PERIOD = 50L;
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
void setup() 
{ 
  Serial.begin(9600); 
  Serial.println(""); //newline

  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);

  headServo.attach(servoData.PIN);
  headServo.write(90);

  delay(1000);
}

// the loop function runs over and over again until power down or reset
// choose which routine to run based on time elapsed since start of cycle
void loop() 
{
  setServo();
  readUltrasonic();
  pidCorrection();
  setMotorsSpeeds();
}

void setServo()
{
  servoTimer1 = millis();

  // poll servo
  if (servoTimer1 > servoTimer2 + servoData.PERIOD && !servoMoving)
  {
    servoMoving = true; //HACK move to servoData
    servoData.sweepHead();
    headServo.write(servoData.getAngle());
    
    //store last time ran
    servoTimer2 = servoTimer1;
  }

  // allow another cycle then, reset to false
  else if (servoTimer1 > servoTimer2 + servoData.PERIOD && servoMoving)
  {
    servoMoving = false;
  }
}

void readUltrasonic()
{
  usTimer1 = millis();

  //send one ping, write to correct index
  if (usTimer1 > usTimer2 + us.PERIOD && !servoMoving)
  {
    //send ping
    us.setPingDistance();

    distances[servoData.getPosition()] = (distances[servoData.getPosition()] + us.getPingDistance()) / 2.0f;
    
    // mark this routine as complete
    if (us.getPingsSent() >= PINGS_PER_ANGLE)
    {
      us.debug();
    }

    //store last time ran
    usTimer2 = usTimer1;
  }
}

void pidCorrection()
{
  pidTimer1 = millis();

  if (pidTimer1 > pidTimer2 + sidePID.PERIOD)
  {

    // side pid corrections
    sidePID.setCurrentError(distances[0], TGT_DISTANCES[0]);
    left1Correction = sidePID.calculatePID(sidePID.getCurrentError());

    sidePID.setCurrentError(distances[1], TGT_DISTANCES[1]);
    left2Correction = sidePID.calculatePID(sidePID.getCurrentError());

    // fwd pid correction
    fwdPID.setCurrentError(distances[4], TGT_DISTANCES[4]);
    fwdCorrection = fwdPID.calculatePID(fwdPID.getCurrentError());

    //data.debug();
    //sidePID.debug();
    //fwdPID.debug();
    debugDistances();
  
    //store last time ran
    pidTimer2 = pidTimer1;
  }
}

//WHEELS ARE FLIPPED
void setMotorsSpeeds()
{
  motorTimer1 = millis();

  if (motorTimer1 > motorTimer2 + MOTOR_PERIOD)
  {
    if (left1Correction < 0)
    {
      leftSpeed += 10;
      rightSpeed -= 10;
    }

    else if (left1Correction > 0)
    {
      leftSpeed -= 10;
      rightSpeed += 10;
    }

    //too close, turn 
    if (fwdCorrection > 0)
    {
      leftSpeed += 10;
      rightSpeed -= 10;
    }

    //too far, speedup
    //else if (fwdCorrection < 0)
    //{
    //  leftSpeed++;
    //  rightSpeed++;
    //}

    // check limits
    if (leftSpeed >= MAX_SPEED) leftSpeed = MAX_SPEED;
    if (rightSpeed >= MAX_SPEED) rightSpeed = MAX_SPEED;
    
    if (leftSpeed <= MIN_SPEED) leftSpeed = MIN_SPEED;
    if (rightSpeed <= MIN_SPEED) rightSpeed = MIN_SPEED;

    // motors are flipped! swap the values
    motors.setSpeeds(rightSpeed, leftSpeed);
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
