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

  //interval period
  const unsigned long PERIOD;

  float KP = 0.0f, KI = 0.0f, KD = 0.0f;

  PID(bool debug, unsigned long period);

  float calculatePID(float currentError);

  void setCurrentError(float currentState, float targetState);

  float getCurrentError();

  void debug();

private:

  // state of the current error vs the target
  float _currentError, prevError;

  //result objects for their respective methods
  float proportional, integral, derivative;

  //sum of all errors
  float KI_total;

  //prevent integral windup
  const float KI_LIMIT = 100.0f;

  void setPrevError(float currentError);

  void setProportional(float currentError);

  void setIntegral(float currentError);

  void setDerivative(float currentError, float prevError);
};

#endif

