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
int ServoData::getAngle() { return _angle; }

// accesser for _position
int ServoData::getPosition() { return _position; }

/**
 * Simple controller to move head
 * reads from headPositions
 * @returns void. sets servo head to a position
 */
void ServoData::sweepHead()
{
  // start at 0 then ascend
  if (sweepingCW)
  {
    _position = (7 + _position + 1) % 7;
    if (_position == 6) sweepingCW = !sweepingCW; // check bounds
  }
  // start at 6 then decend
  else
  {
    _position = (7 + _position - 1) % 7;
    if (_position == 0) sweepingCW = !sweepingCW; // check bounds
  }

  // update the currentAngle
  _angle = HEAD_POSITIONS[_position];
}

//Output Data to serial monitor
void ServoData::debug(char label[])
{
  Serial.println(label);

  Serial.print(" | sweepingCW: "); Serial.print(sweepingCW);
  Serial.print(" | angle: "); Serial.print(_angle);
  Serial.print(" | position: "); Serial.print(_position);
  Serial.print(" | T1: "); Serial.print(timer1);
  Serial.print(" | T2: "); Serial.println(timer2);
}
