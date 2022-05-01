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

  float sideCorrection, fwdcorrection;

  float currentError;

  float KP, KI, KD; //HACK once tuned, make constant

  PID(bool debug);

  float calculatePID(float currentError);

  void debug(char label[]);

private:

  // state of the current error vs the target
  float prevError;

  // result objects for gain
  float KP_Res, KI_Res, KD_Res;

  // sum of all errors
  float KI_total;

  // prevent integral windup
  const float KI_LIMIT = 50.0f;
};
#endif