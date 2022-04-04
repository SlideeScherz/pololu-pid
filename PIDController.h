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

  PID(bool debug, unsigned long period, float kP = 0.6375f, float kI = 0.0f, float kD = 0.0f, float kI_Limit = 500.0f);

  float calculatePID(float currentError);

  void setCurrentError(float currentState, float targetState);

  float getCurrentError();

  void debug();

private:

  // state of the current error vs the target
  float _currentError, prevError;

  //coefficient constants
  const float KP, KI, KD;

  //result objects for their respective methods
  float proportional, integral, derivative;

  //sum of all errors
  float KI_total;

  //prevent integral windup
  const float KI_LIMIT; //TUNE research a KI Limit

  void setPrevError(float currentError);

  void setProportional(float currentError);

  void setIntegral(float currentError);

  void setDerivative(float currentError, float prevError);
};

#endif

