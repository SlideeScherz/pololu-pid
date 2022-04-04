// ServoData.h

#ifndef _SERVODATA_h
#define _SERVODATA_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class ServoData
{
public:

  // hardware pin assignment
  const int PIN;

  unsigned long t2;

  // frequency between sweeps
  const unsigned long PERIOD;

  bool bDebug, ready;

  ServoData(bool debug, unsigned long period, int pin);

  float getAngle();

  int getPosition();

  void sweepHead();

private:

  // curent angle head is aiming in degrees
  float _angle;

  // iterator for HeadPositions array
  int _position;

  //HACK Import global const
  //legal head positions (angles) servo can point
  const float HEAD_POSITIONS[7] = { 135.0f, 120.0f, 105.0f, 90.0f, 75.0f, 60.0f, 45.0f };

  //direction servo is sweeping
  bool sweepingCW;

  void debug();
};

#endif

