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
  currentError = 0.0f;
  prevError = 0.0f;
  KP_Res = 0.0f; 
  KI_Res = 0.0f;
  KD_Res = 0.0f;
}

/**
 * call all methods to output a pid Error correction
 * @param currentState the current smoothed input used to calculate error
 * @param targetState the target used to calculate error
 * @returns PID output
 */
float PID::calculatePID(float errIn)
{
  currentError = errIn;

  KP_Res = KP * currentError;

  // update total error
  KI_total += currentError;

  // prevent windup
  if (KI_total > KI_LIMIT)
    KI_total = KI_LIMIT;

  else if (KI_total < KI_LIMIT * -1.0f)
    KI_total = KI_LIMIT * -1.0f;

  KI_Res = KI * KI_total;

  KD_Res = KD * (currentError - prevError);

  // store error
  prevError = currentError;

  return (KP_Res + KI_Res + KD_Res);
}

// output state to serial monitor
void PID::debug(char label[])
{
  Serial.print(label);
  Serial.print(" | curr error: "); Serial.print(currentError);
  Serial.print(" | prev error: "); Serial.print(prevError);
  Serial.print(" | P: "); Serial.print(KP_Res);
  Serial.print(" | I: "); Serial.print(KI_Res);
  Serial.print(" | D: "); Serial.print(KD_Res);
  Serial.println(" | output : "); Serial.print(KP_Res + KI_Res + KD_Res);
}