// 
// 
// 

#include "DataController.h"

//default constructor
DataController::DataController(bool debug, float period) : PERIOD(period)
{
  bDebug = debug;
  _avgDistance = 0.0f;
  distanceAcc = 0.0f;
}

// member accesser 
float DataController::getAvgDistance()
{
  return _avgDistance;
}

//set _avgDistance and accDistance to 0
void DataController::resetRollingAvg()
{
  _avgDistance = 0.0f;
  distanceAcc = 0.0f;
}

/**
 * Get the rolling average of data passed in
 * read and write the input at the same time for performace
 * Acc the sum as each element is added
 * @param data the distance of the most recent ping
 * @param itr current element in array. Used to divide for rolling average
 * @returns void. sets the _avgDistance member
 */
void DataController::calcRollingAvg(float data, int dataLen)
{
  //accumulatr the distance sum
  distanceAcc += data;

  //get rolling average 
  _avgDistance = distanceAcc / dataLen;
}

// accesser for allDistances
float DataController::allDistancesRead(int atIndex)
{
  return allDistances[atIndex];
}

// modifier for allDistances
void DataController::allDistancesWrite(float data, int atIndex)
{
  allDistances[atIndex] = data;
}

void DataController::setStatus(int status) { _status = status; }

int DataController::getStatus() { return _status; }
