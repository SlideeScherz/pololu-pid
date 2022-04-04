// 
// 
// 

#include "ServoData.h"

/**
 * construct a headservo object
 * @param pin signal pin for headServo
 * @param period interval for scheduler
 */
ServoData::ServoData(bool debug, unsigned long period, int pin) :
  PERIOD(period), PIN(pin)
{
  bDebug = debug;
  sweepingCW = true;
  _position = 0;

  _angle = HEAD_POSITIONS[_position];
}

// accesser for _angle 
float ServoData::getAngle() { return _angle; }

// accesser for _position
int ServoData::getPosition() { return _position; }

/**
 * Simple controller to move head
 * reads from headPositions
 * @returns void. sets servo head to a position
 */
void ServoData::sweepHead()
{
  //HACK make this dynamic
  //check sweep bounds
  if (_position == 6 || _position == 0)
    sweepingCW = !sweepingCW;

  //increment if moving right, else decrement
  _position = (sweepingCW) ? _position++ : _position--;
  
  //update the currentAngle
  _angle = HEAD_POSITIONS[_position];
}

//Output Data to serial monitor
void ServoData::debug()
{
  Serial.print("[servo] ");
  Serial.print("sweepingCW : ");
  Serial.print(sweepingCW);
  Serial.print(" angle: ");
  Serial.print(_angle);
  Serial.print(" position: ");
  Serial.println(_position);
}
