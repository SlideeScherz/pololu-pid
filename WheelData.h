// WheelData.h

#ifndef _WHEELDATA_h
#define _WHEELDATA_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class WheelData
{
public: 

  bool bDebug, encodersFlipped;

  const unsigned long PERIOD;

  //TUNE speed constants
  const float MAX_SPEED = 100.0f;
  const float DEFAULT_SPEED = 75.0f;
  const float MIN_SPEED = 35.0f;

  WheelData(bool debug, unsigned long period);

  void setLeftSpeed(int value);

  int getLeftSpeed();

  void setRightSpeed(int value);

  int getRightSpeed();

  void debug();

private:

  //TODO put all on same line
  //current motor readings, init to default
  float _leftSpeed, _rightSpeed;
};


#endif

