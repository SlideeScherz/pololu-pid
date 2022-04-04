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
  //algo to cycle through photos i used in a react.js project
  //once we hit max, "bounce" the other way
  //start 3. 4, 5, 6, then toggle
  //after 1st pass: start 0. 1, 2, 3, 4, 5, 6 then toggle
  if (sweepingCW)
  {
    _position = (7 + _position + 1) % 7;
    if (_position == 6) sweepingCW = !sweepingCW;
  }
  //start 6. 5, 4, 3, 2, 1, 0 then toggle
  else
  {
    _position = (7 + _position - 1) % 7;
    if (_position == 0) sweepingCW = !sweepingCW;
  }

  //update the currentAngle
  _angle = HEAD_POSITIONS[_position];
}

//Output Data to serial monitor
void ServoData::debug()
{
  Serial.print("servo | ");
  Serial.print(" | sweepingCW: ");
  Serial.print(sweepingCW);
  Serial.print(" | angle: ");
  Serial.print(_angle);
  Serial.print(" | position: ");
  Serial.println(_position);
}
