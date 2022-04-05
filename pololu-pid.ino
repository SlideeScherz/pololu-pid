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
//#include "DataController.h"

using namespace Pololu3piPlus32U4;

/* global data */

constexpr int POS_LEN = 7;
constexpr int PINGS_PER_ANGLE = 20;
constexpr float MAX_DISTANCE = 200.0f;
constexpr float US_MIN_DISTANCE = 2.0f;

Servo headServo;
Motors motors;
Ultrasonic us(false, 20L, 22, 21);

/* These are only for data. Dont manipulate any hardware */
ServoData servoData(false, 20L, 20);
// DataController data();
PID pid(true, 50L); // period a bit after ping

// timers
unsigned long servoTimer1 = 0L, servoTimer2 = 0L;
unsigned long usTimer1 = 0L, usTimer2 = 0L;
unsigned long motorTimer1 = 0L, motorTimer2 = 0L;
unsigned long pidTimer1 = 0L, pidTimer2 = 0L;

//motor data
const unsigned long MOTOR_PERIOD = 50L;
const int MIN_SPEED = 40;
const int DEFAULT_SPEED = 75;
const int MAX_SPEED = 150;
int leftSpeed = MIN_SPEED, rightSpeed = MIN_SPEED;

// target distances 
const float TGT_LEFT = 80.0f;
const float TGT_FWD = 20.0f;

// pid containers
float leftCorrection;
float fwdCorrection;

// container for last average distance of each angle
float distances[POS_LEN] = { };

// the setup function runs once when you press reset or power the board
void setup() 
{ 
  Serial.begin(9600); 

  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);

  servoData.moving = false;

  headServo.attach(servoData.PIN);
  headServo.write(servoData.getAngle());

  pid.KP = 0.6375f;
  pid.KI = 0.0f;
  pid.KD = 2.0f;

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
  if (servoTimer1 > servoTimer2 + 60L && !servoData.moving)
  {
    servoData.moving = true; 
    servoData.sweepHead();
    headServo.write(servoData.getAngle());
    
    // store last time ran
    servoTimer2 = servoTimer1;

    if (servoData.bDebug)
    {
      //Serial.print(headServo.read());
      //Serial.print(" ");
      servoData.debug();
    }
  }
  // allow servo to finish sweep
  else if (servoTimer1 > servoTimer2 + 40L && servoData.moving)
  {
    servoData.moving = false;
    servoTimer2 = servoTimer1;
  }
}

void readUltrasonic()
{
  usTimer1 = millis();

  //send one ping, write to correct index
  if (usTimer1 > usTimer2 + 40L && !servoData.moving)
  {
    //send ping
    us.setPingDistance();

    distances[servoData.getPosition()] = us.getPingDistance();
    
    if (us.bDebug) 
      us.debug();

    //store last time ran
    usTimer2 = usTimer1;
  }
}

void pidCorrection()
{
  pidTimer1 = millis();

  if (pidTimer1 > pidTimer2 + pid.PERIOD && servoData.getPosition() == 0)
  {

    // side pid corrections
    pid.setCurrentError(distances[0], TGT_LEFT);
    leftCorrection = pid.calculatePID(pid.getCurrentError());

    fwdCorrection = TGT_FWD - distances[4];

    //pid.debug();
    //debugDistances();
  
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
    // turn left
    if (leftCorrection < 0) {}

    // turn right 
    else if (leftCorrection > 0) {}

    // check fwd wall
    if (fwdCorrection < 0) {}

    // check limits
    if (rightSpeed >= MAX_SPEED) rightSpeed = MAX_SPEED;
    else if (rightSpeed <= MIN_SPEED) rightSpeed = MIN_SPEED;

    // motors are flipped! swap the values
    motors.setSpeeds(rightSpeed, DEFAULT_SPEED);

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
