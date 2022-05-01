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

// HACK make these global 
const int POS_LEN = 7;

// TODO evalute if needed
const int PINGS_PER_POS = 20;

// TODO move to US controller
// us reading data and pid limits
const float US_MIN_DISTANCE = 2.0f, MAX_DISTANCE = 200.0f;

// pin assignments 
const int US_TRIG_PIN = 22, US_ECHO_PIN = 21, SERVO_PIN = 20;

/* hardware init */
Servo headServo;
Motors motors;
Ultrasonic us(false, 10L, US_TRIG_PIN, US_ECHO_PIN);
PID pid(false, 20L); 

/* head servo data */
// milliseconds interval for scheduler
const unsigned long SERVO_PERIOD = 50UL;

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
const int HEAD_POSITIONS[7] = { 135, 120, 105, 90, 75, 60, 45 };

// position readings from each angle
int distances[7] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL };

// motor data
const unsigned long MOTOR_PERIOD = 50L;
unsigned long motorTimer1 = 0L, motorTimer2 = 0L;

const int MIN_SPEED = 60;
const int DEFAULT_SPEED = 100;
const int MAX_SPEED = 150;

int leftSpeed = DEFAULT_SPEED, rightSpeed = DEFAULT_SPEED;

// target distances 
const float TGT_LEFT = 15.0f;
const float TGT_FWD = 10.0f;

// the setup function runs once when you press reset or power the board
void setup() 
{ 
  Serial.begin(9600); 

  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);

  headServo.attach(SERVO_PIN);
  headServo.write(servoAngle);

  pid.KP = 2.0f;
  pid.KI = 2.0f;
  pid.KD = 2.0f;

  delay(1000);
}

// the loop function runs over and over again until power down or reset
void loop() 
{
  //setServo(); //TEMPTEST
  readUltrasonic();
  pidCorrection();
  //setMotorsSpeeds();
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

void readUltrasonic()
{
  us.timer1 = millis();

  //send one ping, write to correct index
  if (us.timer1 > us.timer2 + us.PERIOD && !servoMoving)
  {
    //send ping
    us.setPingDistance();

    distances[servoPos] = us.getPingDistance();

    if (us.bDebug) us.debug("Ultrasonic");
    
    //debugDistances();

    //store last time ran
    us.timer2 = us.timer1;
  }
}

void pidCorrection()
{
  pid.timer1 = millis();

  if (pid.timer1 > pid.timer2 + pid.PERIOD && servoPos == 0)
  {
    // side pid corrections
    pid.setCurrentError(distances[servoPos], TGT_LEFT); //HACK make dynamic
    pid.sideCorrection = pid.calculatePID(pid.getCurrentError());

    //pid.fwdCorrection = TGT_FWD - distances[4];

    if(pid.bDebug) pid.debug("side pid");
    
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

/**
 * Simple controller to move head
 * reads from headPositions
 * @returns void. sets servo head to a position
 */
void sweepHead()
{
  // start at 0 then ascend
  if (sweepingClockwise)
  {
    servoPos = (7 + servoPos + 1) % 7;
    if (servoPos == 6) sweepingClockwise = !sweepingClockwise; // check bounds
  }
  // start at 6 then decend
  else
  {
    servoPos = (7 + servoPos - 1) % 7;
    if (servoPos == 0) sweepingClockwise = !sweepingClockwise; // check bounds
  }

  // TODO may not be used.
  // update the currentAngle
  servoAngle = HEAD_POSITIONS[servoPos];
}

//Output Data to serial monitor
void headServoDebug(char label[])
{
  Serial.println(label);

  Serial.print(" | sweepingCW: "); Serial.print(sweepingClockwise);
  Serial.print(" | angle: "); Serial.print(servoAngle);
  Serial.print(" | position: "); Serial.print(servoPos);
  Serial.print(" | T1: "); Serial.print(servoTimer1);
  Serial.println(" | T2: "); Serial.println(servoTimer2);
}

/*
 * set the LEDS to on or off.
 * @param (color)State 0 (off) or 1 (on)
 * @returns void. Sets the pololu LED pins
 */
void setLEDs(int yellowState, int greenState, int redState) {
  ledYellow(yellowState);
  ledGreen(greenState);
  ledRed(redState);
}
