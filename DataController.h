// DataController.h

#ifndef _DATACONTROLLER_h
#define _DATACONTROLLER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class DataController
{
public:

  bool bDebug, ready;

  int wallTarget;

  const unsigned long PERIOD;

  DataController(bool debug, unsigned long period);

  float getAvgDistance();
  
  void resetRollingAvg();

  void calcRollingAvg(float data, int dataLen);

private:

  // used for deriving the rolling average of distances read by US
  float _avgDistance, distanceAcc;
};
#endif