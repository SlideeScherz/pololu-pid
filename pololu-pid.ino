/*
 Name:		pololu_pid.ino
 Created:	4/3/2022 4:00:33 PM
 Author:	Scott Scherzer
*/

#include <Servo.h>
#include <Pololu3piPlus32U4.h>
#include "constants.h"
#include "UltrasonicController.h"
#include "PIDController.h"
#include "ServoData.h"

using namespace Pololu3piPlus32U4;

/* hardware init */
Servo headServo;
Motors motors;
Ultrasonic us(false, 20L, constants::US_TRIG_PIN, constants::US_ECHO_PIN);
ServoData servoData(false, 20L, constants::SERVO_PIN);
PID pid(true, 20L); 

// hack, move to class once tuned
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

// container for last average distance of each angle
float distances[constants::POS_LEN] = { };

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
  setMotorsSpeeds();
}

void setServo()
{
  servoData.timer1 = millis();

  // poll servo
  if (servoData.timer1 > servoData.timer2 + 60L && !servoData.moving)
  {
    servoData.moving = true; 
    servoData.sweepHead();
    headServo.write(servoData.getAngle());
    
    // store last time ran
    servoData.timer2 = servoData.timer1;

    if (servoData.bDebug) servoData.debug();
  }

  // allow servo to finish sweep
  else if (servoData.timer1 > servoData.timer2 + 40L && servoData.moving)
  {
    servoData.moving = false;
    servoData.timer2 = servoData.timer1;
  }
}

void readUltrasonic()
{
  us.timer1 = millis();

  //send one ping, write to correct index
  if (us.timer1 > us.timer2 + 40L && !servoData.moving)
  {
    //send ping
    us.setPingDistance();

    //distances[servoData.getPosition()] = us.getPingDistance();
    distances[0] = us.getPingDistance();

    if (us.bDebug) us.debug();
    
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
    pid.setCurrentError(distances[0], TGT_LEFT); //HACK make dynamic
    pid.sideCorrection = pid.calculatePID(pid.getCurrentError());

    //pid.fwdCorrection = TGT_FWD - distances[4];

    if(pid.bDebug) pid.debug();
    
    //store last time ran
    pid.timer2 = pid.timer1;
  }
}

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

void debugDistances()
{
  Serial.print("dist | ");
  
  for (int distItr = 0; distItr < constants::POS_LEN; distItr++)
  {
    Serial.print(distances[distItr]); 
    Serial.print(" | ");
  }
  
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
