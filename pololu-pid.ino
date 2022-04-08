/*
 Name:		pololu_pid.ino
 Created:	4/3/2022 4:00:33 PM
 Author:	Scott Scherzer
*/

#include <Servo.h>
#include <Pololu3piPlus32U4.h>
#include "DataController.h"
#include "UltrasonicController.h"
#include "PIDController.h"
#include "ServoData.h"

using namespace Pololu3piPlus32U4;

// HACK make these global 
const int POS_LEN = 7, PINGS_PER_POS = 20;

// us reading data and pid limits
const float US_MIN_DISTANCE = 2.0f, MAX_DISTANCE = 200.0f;

// pin assignments 
const int US_TRIG_PIN = 22, US_ECHO_PIN = 21, SERVO_PIN = 20;

/* hardware init */
Servo headServo;
Motors motors;
DataController data;
Ultrasonic us(false, 10L, US_TRIG_PIN, US_ECHO_PIN);
ServoData servoData(false, 50L, SERVO_PIN);
PID pid(false, 20L); 

/*
//TODO timer1 is redundant for all classes
Use use timer = millis() in main, and compare to each class last ran
*/

// HACK, move to class once tuned
// motor data
const unsigned long MOTOR_PERIOD = 50L;
const int MIN_SPEED = 60;
const int DEFAULT_SPEED = 100;
const int MAX_SPEED = 150;

int leftSpeed = MIN_SPEED, rightSpeed = MIN_SPEED;
unsigned long motorTimer1 = 0L, motorTimer2 = 0L;

// target distances 
const float TGT_LEFT = 15.0f;
const float TGT_FWD = 10.0f;

// the setup function runs once when you press reset or power the board
void setup() 
{ 
  Serial.begin(9600); 

  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);

  //servoData.moving = false;

  //headServo.attach(servoData.PIN);
  //headServo.write(servoData.getAngle());

  pid.KP = 2.0f;
  pid.KI = 2.0f;
  pid.KD = 2.0f;

  delay(1000);
}

// the loop function runs over and over again until power down or reset
// choose which routine to run based on time elapsed since start of cycle
void loop() 
{
  //setServo(); //TEMPTEST
  readUltrasonic();
  pidCorrection();
  //setMotorsSpeeds();
}

void setServo()
{
  servoData.timer1 = millis();

  // poll servo
  if (servoData.timer1 > servoData.timer2 + servoData.PERIOD && !servoData.moving)
  {
    servoData.moving = true; 
    servoData.sweepHead();
    headServo.write(servoData.getAngle());
    
    // store last time ran
    servoData.timer2 = servoData.timer1;
  }

  // allow servo to finish sweep
  // TUNE wait period
  else if (servoData.timer1 > servoData.timer2 + 40L && servoData.moving)
  {
    servoData.moving = false;
    servoData.timer2 = servoData.timer1;

    if (servoData.bDebug) servoData.debug("Head Servo");
  }
}

void readUltrasonic()
{
  us.timer1 = millis();

  //send one ping, write to correct index
  if (us.timer1 > us.timer2 + us.PERIOD && !servoData.moving)
  {
    //send ping
    us.setPingDistance();

    //distances[servoData.getPosition()] = us.getPingDistance();
    data.distances[0] = us.getPingDistance();

    if (us.bDebug) us.debug("Ultrasonic");
    
    //debugDistances();

    //store last time ran
    us.timer2 = us.timer1;
  }
}

void pidCorrection()
{
  pid.timer1 = millis();

  if (pid.timer1 > pid.timer2 + pid.PERIOD && servoData.getPosition() == 0)
  {
    // side pid corrections
    pid.setCurrentError(data.distances[0], TGT_LEFT); //HACK make dynamic
    pid.sideCorrection = pid.calculatePID(pid.getCurrentError());

    //pid.fwdCorrection = TGT_FWD - distances[4];

    if(pid.bDebug) pid.debug("side pid");
    
    //store last time ran
    pid.timer2 = pid.timer1;
  }
}

//TODO move to class
void setMotorsSpeeds()
{
  motorTimer1 = millis();

  rightSpeed = pid.sideCorrection;

  // check limits
  if (rightSpeed >= MAX_SPEED) rightSpeed = MAX_SPEED;
  else if (rightSpeed <= MIN_SPEED) rightSpeed = MIN_SPEED;

  if (leftSpeed >= MAX_SPEED) leftSpeed = MAX_SPEED;
  else if (leftSpeed <= MIN_SPEED) leftSpeed = MIN_SPEED;

  // motors are flipped! swap the values
  motors.setSpeeds(rightSpeed, leftSpeed);

  motorTimer2 = motorTimer1;
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
