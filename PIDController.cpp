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
PID::PID(bool debug, unsigned long period, float kP = 0.6375f, float kI = 0.0f, float kD = 0.0f) :
  PERIOD(period), KP(kP), KI(kI), KD(kD)
{
  bDebug = debug;
 
  // defaults 
  _currentError = 0.0f;
  prevError = 0.0f;
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

  setPrevError(currentError);
  return proportional + integral + derivative;
}

/**
 * Read From UltraSonic, smooth data then calculate the error
 * @param currentState the current smoothed input used to calculate error
 * @param targetState the target used to calculate error
 * @returns void. Set the current error private member
 */
void PID::setCurrentError(float currentState, float targetState)
{
  _currentError = targetState - currentState;
}

float PID::getCurrentError() { return _currentError; }

// output state to serial monitor
void PID::debug()
{
  Serial.print("[PID] ");

  Serial.print("error: ");
  Serial.print(_currentError);

  Serial.print(" P: ");
  Serial.print(proportional);

  Serial.print(" I: ");
  Serial.print(integral);

  Serial.print(" D: ");
  Serial.print(derivative);

  Serial.print(" PID=> : ");
  Serial.println(proportional + integral + derivative);
}

/**  
 * write the current error to the previous error
 * Called once the pid has completed
 * @param currentError current error passed in from setCurrentError
 * @returns void. Set the prevError private member
 */
void PID::setPrevError(float currentError) { prevError = currentError; }

// set the proportional error from the current error
void PID::setProportional(float currentError)
{
  proportional = KP * currentError;
}

// manage steady state error with the integral
void PID::setIntegral(float currentError)
{
  KI_total += currentError;

  if (KI_total > KI_LIMIT)
  {
    KI_total = KI_LIMIT;
  }

  integral = KI * KI_total;
}

// get the delta of the last two errors to account for future error
void PID::setDerivative(float currentError, float prevError)
{
  //store the last error after!
  derivative = KD * (currentError - prevError);
}
