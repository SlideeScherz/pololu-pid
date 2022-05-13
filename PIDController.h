// PID.h

#ifndef _PIDCONTROLLER_h
#define _PIDCONTROLLER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

//Takes input data and returns a suggested error correction
class PID
{
public:

  bool bDebug;

  double gain;

  double currentError;

  double KP, KI, KD; 

  PID(bool debug);

  double calculatePID(double currentError);

  void debug(char label[]);

private:

  // state of the current error vs the target
  double prevError;

  // result objects for gain
  double kpRes, kiRes, kdRes;

  // sum of all errors
  double kiTotal;

  // prevent integral windup
  const double KI_LIMIT = 50.0;
};
#endif