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

  // output to serial 
  bool bDebug;

  ServoData(bool debug, unsigned long period, int pin);

  float getAngle();

  float getPosition();

  void sweepHead();

private:

  // curent angle head is aiming in degrees
  float _angle;

  // iterator for HeadPositions array
  int _position;

  //len of array 
  const int headPositionsLen = 7;

  //legal head positions (angles) servo can point
  const float headPositions[7] = { 135.0f, 120.0f, 105.0f, 90.0f, 75.0f, 60.0f, 45.0f };

  //direction servo is sweeping
  bool sweepingCW;

  void debug();
};

#endif

