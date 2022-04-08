// constants.h

#ifndef _CONSTANTS_h
#define _CONSTANTS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

namespace constants
{
  // us ping containers
  extern constexpr int POS_LEN = 7;
  extern constexpr int PINGS_PER_POS = 20;

  // us reading data and pid limits
  extern constexpr float MAX_DISTANCE = 200.0f;
  extern constexpr float US_MIN_DISTANCE = 2.0f;

  // pin assignments 
  extern constexpr int US_TRIG_PIN = 22;
  extern constexpr int US_ECHO_PIN = 21;
  extern constexpr int SERVO_PIN = 20;
}
#endif