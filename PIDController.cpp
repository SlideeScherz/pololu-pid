// 
// 
// 

#include "PIDController.h"

/**
 * Constructor for PID
 * @param debug enable or disable serial logging
 */
PID::PID(bool debug)
{
  bDebug = debug;
 
  // defaults 
  currentError = 0.0;
  prevError = 0.0;
  kpRes = 0.0; 
  kiRes = 0.0;
  kdRes = 0.0;
}

/**
 * call all methods to output a pid Error correction
 * @param currentState the current smoothed input used to calculate error
 * @param targetState the target used to calculate error
 * @returns PID output
 */
double PID::calculatePID(double errIn)
{
  currentError = errIn;

  kpRes = KP * currentError;

  // update total error
  kiTotal += currentError;

  // prevent windup
  if (kiTotal > KI_LIMIT)
    kiRes = KI * KI_LIMIT;

  else
    kiRes = KI * kiTotal;

  kdRes = KD * (currentError - prevError);

  // store error
  prevError = currentError;

  return (kpRes + kiRes + kdRes);
}

// output state to serial monitor
void PID::debug(char label[])
{
  Serial.print(label);
  Serial.print(" | curr error: "); Serial.print(currentError);
  Serial.print(" | prev error: "); Serial.print(prevError);
  Serial.print(" | P: "); Serial.print(kpRes);
  Serial.print(" | I: "); Serial.print(kiRes);
  Serial.print(" | D: "); Serial.print(kdRes);
  Serial.print(" | gain : "); Serial.print(gain);
  Serial.print("\n");
}