/*
 Name:		pololu_pid.ino
 Created:	4/3/2022 4:00:33 PM
 Author:	Scott Scherzer
*/

#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include "UltrasonicController.h"
#include "PIDController.h"

using namespace Pololu3piPlus32U4;

// pin assignments 
const uint8_t US_TRIG_PIN = 22, US_ECHO_PIN = 21, SERVO_PIN = 20;

constexpr int POS_LEN = 7;

const double SETPOINT = 15.0;

// TODO move to US controller
// us reading data and pid limits
const double US_MIN_DISTANCE = 2.0, MAX_DISTANCE = 200.0;

/* hardware init */
Servo headServo;
Motors motors;
Ultrasonic us(false, 10ul, US_TRIG_PIN, US_ECHO_PIN);
PID pid(false); 

/* head servo data */
// milliseconds interval for scheduler
const unsigned long SERVO_PERIOD = 50ul;

// timers for servo
unsigned long servoTimer1, servoTimer2;

// angle servo is currently facing
int servoAngle = 90;

// index of HEAD_POSITIONS array 
int servoPos = 3;

// debug switch 
bool bDebugHeadServo = false; 

// logic for servo sweeping right or left
bool sweepingClockwise = true;

// logic to stop US from sending pings if US is moving
bool servoMoving = false;

// legal head positions (angles) servo can point
const int HEAD_POSITIONS[POS_LEN] = { 45, 60, 75, 90, 105, 120, 135 };

// position readings from each angle
double distances[POS_LEN] = { };

// motor data
const unsigned long MOTOR_PERIOD = 50ul;
unsigned long motorTimer1, motorTimer2;

const int MIN_SPEED = 50;
const int BASE_SPEED = 100;
const int MAX_SPEED = 150;

int leftSpeed = BASE_SPEED, rightSpeed = BASE_SPEED;

// the setup function runs once when you press reset or power the board
void setup() 
{ 
  Serial.begin(9600); 

  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);

  headServo.attach(SERVO_PIN);
  headServo.write(servoAngle);

  pid.KP = 4.0;
  pid.KI = 0.5;
  pid.KD = 2.0;

  delay(1000);
}

// the loop function runs over and over again until power down or reset
void loop() 
{
  setServo(); 
  readUltrasonic();
  setMotorsSpeeds();
}

// util methods
int handleLimit(int value, int min, int max)
{
  if (value >= max) 
    value = max;
  else if (value <= min) 
    value = min;
}

void setServo()
{
  servoTimer1 = millis();

  // poll servo
  if (servoTimer1 > servoTimer2 + SERVO_PERIOD && !servoMoving)
  {
    servoMoving = true;
    sweepHead();
    headServo.write(servoAngle);
    
    // store last time ran
    servoTimer2 = servoTimer1;
  }

  // allow servo to finish sweep
  // TUNE wait period
  else if (servoTimer1 > servoTimer2 + SERVO_PERIOD && servoMoving)
  {
    servoMoving = false;
    servoTimer2 = servoTimer1;

    if (bDebugHeadServo) headServoDebug("Head Servo");
  }
}

/**
 * Simple controller to move head
 * reads from headPositions
 * @returns void. sets servo head to a position
 */
void sweepHead()
{
  // toggle direction
  if (servoPos == 6 || servoPos == 0)
    sweepingClockwise = !sweepingClockwise;

  // start at 0 then ascend
  if (sweepingClockwise)
    servoPos = (7 + servoPos + 1) % 7;

  // start at 6 then decend
  else
    servoPos = (7 + servoPos - 1) % 7;

  // update the currentAngle
  servoAngle = HEAD_POSITIONS[servoPos];
}

void readUltrasonic()
{
  us.timer1 = millis();

  //send one ping, write to correct index
  if (us.timer1 > us.timer2 + us.PERIOD && !servoMoving)
  {
    //send ping
    distances[servoPos] = us.sendPing();

    if (us.bDebug) us.debug("Ultrasonic");
    
    //store last time ran
    us.timer2 = us.timer1;
  }
}

void setMotorsSpeeds()
{
  motorTimer1 = millis();

  if (motorTimer1 > motorTimer2 + MOTOR_PERIOD)
  {
    // side pid corrections
    pid.currentError = distances[servoPos] - SETPOINT; 

    pid.gain = pid.calculatePID(pid.currentError);

    leftSpeed = BASE_SPEED - pid.gain;
    rightSpeed = BASE_SPEED + pid.gain;

    leftSpeed = handleLimit(leftSpeed, MIN_SPEED, MAX_SPEED);
    rightSpeed = handleLimit(rightSpeed, MIN_SPEED, MAX_SPEED);

    motors.setSpeeds(leftSpeed, rightSpeed);

    if (pid.bDebug) pid.debug("side pid");

    motorTimer2 = motorTimer1;
  }
}

// output data to serial monitor
void headServoDebug(char label[])
{
  Serial.println(label);

  Serial.print(" | sweepingCW: "); Serial.print(sweepingClockwise);
  Serial.print(" | angle: "); Serial.print(servoAngle);
  Serial.print(" | position: "); Serial.print(servoPos);
  Serial.print(" | T1: "); Serial.print(servoTimer1);
  Serial.println(" | T2: "); Serial.println(servoTimer2);
}

