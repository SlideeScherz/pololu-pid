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
Ultrasonic::Ultrasonic(bool debug, unsigned long period, uint8_t triggerPin, uint8_t echoPin) :
  PERIOD(period), TRIG_PIN(triggerPin), ECHO_PIN(echoPin)
{
  bDebug = debug;

  //assign pins to I/O
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

/**
 * get distance using sensor state to determine ping
 * Called by setPingDistance(), written to _pingDistance
 * @returns pingDistance.
 */
double Ultrasonic::sendPing()
{
  // set trigger pin to low voltage
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // activate trigger pin for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  // clear trigger pin
  digitalWrite(TRIG_PIN, LOW);

  // read echo pin and read second wave travel time
  pingTimeDuration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT);

  // calculate round trip time of flight
  return (pingTimeDuration * SPEED_OF_SOUND / 2);
}

/**
 * output state to serial console
 * you must first call setPingDistance()
 * @returns void. Prints to console
 */
void Ultrasonic::debug(char label[])
{
  Serial.print(label);
  Serial.print(" | TOF: "); Serial.print(pingTimeDuration);
  Serial.print(" | distance: "); Serial.print(pingDistance);
  Serial.print(" | T1: "); Serial.print(timer1);
  Serial.println(" | T2: "); Serial.println(timer2);
}