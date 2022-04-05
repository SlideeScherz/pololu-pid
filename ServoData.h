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

  // frequency between sweeps
  const unsigned long PERIOD;

  bool bDebug, moving;

  ServoData(bool debug, unsigned long period, int pin);

  int getAngle();

  int getPosition();

  void sweepHead();

  void debug();

private:

  // curent angle head is aiming in degrees
  float _angle;

  // iterator for HeadPositions array
  int _position;

  //HACK Import global const
  //legal head positions (angles) servo can point
  const int HEAD_POSITIONS[7] = { 135, 120, 105, 90, 75, 60, 45 };

  //direction servo is sweeping
  bool sweepingCW;
};

#endif

