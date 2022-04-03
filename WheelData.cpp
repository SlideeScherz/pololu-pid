// 
// 
// 

#include "WheelData.h"

WheelData::WheelData(bool debug, unsigned long period) : PERIOD(period)
{
  bDebug = debug;
  _leftSpeed = DEFAULT_SPEED;
  _rightSpeed = DEFAULT_SPEED;
}

// public modifier for leftSpeed
void WheelData::setLeftSpeed(int value) { _leftSpeed = value; }

// public accesser to leftSpeed
int WheelData::getLeftSpeed() { return _leftSpeed; }

// public modifier for rightSpeed
void WheelData::setRightSpeed(int value) { _rightSpeed = value; }

// public accesser to rightSpeed
int WheelData::getRightSpeed() { return _rightSpeed; }

//debug motor speed data
void WheelData::debug()
{
  Serial.print("[Wheels] ");
  Serial.print("L: ");
  Serial.print(_leftSpeed);
  Serial.print(" R: ");
  Serial.print(_rightSpeed);
}