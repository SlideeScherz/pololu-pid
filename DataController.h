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

	bool bDebug;

  DataController(bool debug);

  float getAvgDistance();
  
  void resetRollingAvg();

  void calcRollingAvg(float data, int currentIndex);

  float allDistancesRead(int atIndex);

  void allDistancesWrite(float data, int atIndex);

private:

  //Used for deriving the rolling average of distances read by US
  float _avgDistance, distanceAcc;

  //TODO move to .ino and make extern
  //Farthest distance we want to read
  const float MAX_DISTANCE = 400.0f;

  //TODO add 7 to extern const
  //container for last average distance of each angle
  float allDistances[7] = { };
};

#endif