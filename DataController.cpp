// 
// 
// 

#include "DataController.h"

DataController::DataController()
{
  // reset distances //TODO may become a method
  for (int distItr = 0; distItr < 7; distItr++) { distances[distItr] = NULL; }
}

void DataController::debugDistances(char label[])
{
  Serial.println(label);

  for (int distItr = 0; distItr < 7; distItr++)
  {
    Serial.print(distances[distItr]);
    Serial.print(" | ");
  }

  Serial.println(" | ");
}
