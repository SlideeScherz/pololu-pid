// UltrasonicController.h

#ifndef _ULTRASONICCONTROLLER_h
#define _ULTRASONICCONTROLLER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

// ultrasonic sensor on top of a moving servo
class Ultrasonic
{
public:

  bool bDebug;

  // milliseconds interval for scheduler
  const unsigned long PERIOD;

  unsigned long timer1, timer2;

  // hardware pins to MOBO
  const uint8_t TRIG_PIN, ECHO_PIN;

  // time in MICROSECONDS of round trip flight for ping to return 
  unsigned long pingTimeDuration, pingDistance;

  Ultrasonic(bool debug, unsigned long period, uint8_t triggerPin, uint8_t echoPin);

  float sendPing();

  void debug(char label[]);

private:

  //used in pingDistance calculation
  const float SPEED_OF_SOUND = 0.034f;

  /**
   * MICROSECONDS to wait for ping to return
   * max distance = (speed of sound)x
   * 400cm = (0.034 m/s)x 
   * x = 11764.705882 //TUNE
   */
  const unsigned long ECHO_TIMEOUT = 38000UL;

};
#endif
