// 
// 
// 

#include "UltrasonicController.h"

/**
 * Construct an ultrasonic object
 * @param debug enable or disable debugging
 * @param triggerPin mobo pin assignment for input
 * @param echoPin mobo pin assignment for output
 * @param period schedular interval
 */
Ultrasonic::Ultrasonic(bool debug, unsigned long period, int triggerPin, int echoPin) :
  PERIOD(period), TRIG_PIN(triggerPin), ECHO_PIN(echoPin)
{
  //assign pins to I/O
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  //set defaults
  t2 = 0L;
  bDebug = debug;
}

// call sendPing and assign the distance to pingDistance
void Ultrasonic::setPingDistance() { _pingDistance = sendPing(); }

// access private member ping distance
float Ultrasonic::getPingDistance() { return _pingDistance; }

// public modifier
void Ultrasonic::setStatus(int value) { _status = value; }

//public acesser
int Ultrasonic::getStatus() { return _status; }

// public modifier
void Ultrasonic::setPingsSent(int value) { _pingsSent = value; }

//public acesser
int Ultrasonic::getPingsSent() { return _pingsSent; }


/**
 * output state to serial console
 * you must first call setPingDistance()
 * @returns void. Prints to console
 */
void Ultrasonic::debug()
{
  Serial.print("[US] ");
  Serial.print("TOF: ");
  Serial.print(pingTimeDuration);
  Serial.print(" CM: ");
  Serial.println(_pingDistance);
}

/**
 * set trigger pin to high voltage
 * @param microSecondDelay optional
 */
void Ultrasonic::chargeTriggerPin(unsigned int microSecondDelay = 0)
{
  digitalWrite(TRIG_PIN, HIGH);

  //read optional parameter
  if (microSecondDelay != 0)
    delayMicroseconds(microSecondDelay);
}

/**
 * set trigger pin to low voltage
 * @param microSecondDelay optional
 */
void Ultrasonic::clearTriggerPin(unsigned int microSecondDelay = 0)
{
  digitalWrite(TRIG_PIN, LOW);

  //read optional parameter
  if (microSecondDelay != 0)
    delayMicroseconds(microSecondDelay);
}

/**
 * get distance using sensor state to determine ping
 * Called by setPingDistance(), written to _pingDistance
 * @returns pingDistance.
 */
float Ultrasonic::sendPing()
{
  // Clear Trigger pin
  clearTriggerPin(2);

  // Activate Trigger pin for 10 microseconds
  chargeTriggerPin(10);

  // Clear Trigger pin
  clearTriggerPin();

  // Read echo pin and read second wave travel time
  pingTimeDuration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT);

  // inc
  _pingsSent++;

  // calculate round trip time of flight
  return (pingTimeDuration * SPEED_OF_SOUND / 2.0f);
}

