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

	bool debug;

	//const unsigned long PERIOD;

	//unsigned long timer1, timer2;
	
	float distances[7];
	
	DataController();

	void debugDistances(char label[]);

	// TODO data smoothing methods

private:

	// TODO make private and use set get to access/ modify
	// float distances[7];


};
#endif