// UltrasonicController.h

#ifndef _ULTRASONICCONTROLLER_h
#define _ULTRASONICCONTROLLER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

//Ultrasonic sensor on top of a moving servo
class Ultrasonic
{
public:

  bool bDebug;

  // milliseconds interval for scheduler
  const unsigned long PERIOD; 

  //Hardware pins to MOBO
  const int TRIG_PIN, ECHO_PIN;

  Ultrasonic(bool debug, unsigned long period, int triggerPin, int echoPin);

  void setPingDistance();

  float getPingDistance();

  void debug();

private:

  //round trip distance of a ping derived from timeDiration, in CM
  float _pingDistance;

  //time in MICROSECONDS of round trip flight for ping to return 
  unsigned long pingTimeDuration;

  //used in pingDistance calculation
  const float SPEED_OF_SOUND = 0.034f;

  /**
   * MICROSECONDS to wait for ping to return
   * max distance = (speed of sound)x
   * 400cm = (0.034 m/s)x 
   * x = 11764.705882 //TUNE
   */
  const unsigned long ECHO_TIMEOUT = 38000L;

  void chargeTriggerPin(unsigned int microSecondDelay = 0);

  void clearTriggerPin(unsigned int microSecondDelay = 0);

  float sendPing();
};
#endif
