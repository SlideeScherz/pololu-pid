// 
// 
// 

#include "PIDController.h"

/**
 * Constructor for PID
 * @param period millisecond interval for PERIOD used in scheduler
 * @param kP (optional) the proportional coefficient. Default: 0.6375
 * @param kI (optional) the integral coefficient. Default: 0
 * @param kD (optional) the derivative coefficient. Default: 0
 * @param kiLimit (optional) Max KI to prevent integral windup. Default: 100
 */
PID::PID(bool debug, unsigned long period) :
  PERIOD(period)
{
  bDebug = debug;
 
  // defaults 
  _currentError = 0.0f;
  prevError = 0.0f;
  proportional = 0.0f; 
  integral = 0.0f;
  derivative = 0.0f;
}

/**
 * call all methods to output a pid Error correction
 * @param currentState the current smoothed input used to calculate error
 * @param targetState the target used to calculate error
 * @returns PID output
 */
float PID::calculatePID(float currentError)
{
  setProportional(currentError);
  setIntegral(currentError);
  setDerivative(currentError, prevError);
  setPrevError(_currentError);

  return (proportional + integral + derivative);
}

/**
 * Read From UltraSonic, smooth data then calculate the error
 * @param currentState the current smoothed input used to calculate error
 * @param targetState the target used to calculate error
 * @returns void. Set the current error private member
 */
void PID::setCurrentError(float currentState, float targetState) { _currentError = targetState - currentState; }

// public accesser for current error
float PID::getCurrentError() { return _currentError; }

// output state to serial monitor
void PID::debug(char label[])
{
  Serial.println(label);

  Serial.print(" | curr error: "); Serial.print(_currentError);
  Serial.print(" | prev error: "); Serial.print(_currentError);
  Serial.print(" | P: "); Serial.print(proportional);
  Serial.print(" | I: "); Serial.print(integral);
  Serial.print(" | D: "); Serial.print(derivative);
  Serial.print(" | output : "); Serial.print(proportional + integral + derivative);
  Serial.print(" | T1: "); Serial.print(timer1);
  Serial.print(" | T2: "); Serial.println(timer2);
}

/**
 * write the current error to the previous error every cycle
 * @param currentError the private current error member
 */
void PID::setPrevError(float currentError) { prevError = _currentError; }

// set the proportional error from the current error
void PID::setProportional(float currentError) { proportional = KP * currentError; }

// manage steady state error with the integral
void PID::setIntegral(float currentError)
{
  KI_total += currentError;

  integral = KI * KI_total;

  if (integral > KI_LIMIT)
    integral = KI_LIMIT;
  
  else if (integral < KI_LIMIT * -1.0f)
    integral = KI_LIMIT * -1.0f;
}

// get the delta of the last two errors to account for future error
// you must set the previous error after calling this.
void PID::setDerivative(float currentError, float prevError) { derivative = KD * (currentError - prevError); }
