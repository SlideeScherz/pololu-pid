/*
 Name:		pololu_pid.ino
 Created:	4/3/2022 4:00:33 PM
 Author:	Scott Scherzer
*/

#include <Pololu3piPlus32U4OLED.h>
#include <Pololu3piPlus32U4Motors.h>
#include <Pololu3piPlus32U4Buzzer.h>
#include <Pololu3piPlus32U4.h>
#include "UltrasonicController.h"
#include "PIDController.h"
#include "ServoData.h"
#include "DataController.h"
#include "WheelData.h"

using namespace Pololu3piPlus32U4;

Ultrasonic us(true, 100L, 22, 21);
PID sidePID(false, 60L); // period a bit after ping
PID fwdPID(false, 100L); // period a bit after ping
DataController data(false);

/* These are only for data. Dont manipulate any hardware */
ServoData servoData(false, 20L, 20);
WheelData wheels(false, 100L);

/*
 * scheduler timing
 * t1: time at the start of the cycle. Updated when all routines finished
 * t2: time elapsed since t1 began. to be called each loop
 */
unsigned long t1, t2;

//Reset the scheduler, and rewrite t1 at this period
const unsigned long RESET_SCHEDULER = 1000L;

/* environment data */

//TUNE LEN
//wall constants
enum eWall { NONE, LEFT, CENTER, RIGHT, LEN = 20 };

// target distances in respect to the angles in servo.headPositions[]
const float TGT_DISTANCES[7] = { 80.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f , 0.0f };


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600); 
  
  Serial.println(); //newline

  //temp us pin test
  Serial.print(us.TRIG_PIN); Serial.print(" "); Serial.println(us.ECHO_PIN);

  //initialize timer1
  t1 = millis();

  checkBatteries();
}

// the loop function runs over and over again until power down or reset
void loop() {
  
}

/*
 * Chose which routine to run based on time elapsed since start of cycle
 * @param timeBegan is the current timestamp. Updated at the end of every cycle.
 * @returns void. Scheduler will call the correct method
 */
void scheduler(unsigned long timeBegan)
{
  //get current time
  t2 = millis() - timeBegan;

  //send one ping, write to correct index
  if (t2 == us.PERIOD)
  {
    //send 20 pings. Each takes ~50 microseconds to complete
    for (int pings = 0; pings < 20; pings++)
    {
      //send ping
      us.setPingDistance();

      //fetch from US, and get a moving avg from data
      data.calcRollingAvg(us.getPingDistance(), pings);
    }

    //HACK pass in servoItr
    data.allDistancesWrite(data.getAvgDistance(), 0);
    data.resetRollingAvg();
  }

  //Last step. Reset timer1
  else if (t2 >= RESET_SCHEDULER)
  {
    t1 = millis();
  }
}

/*
 * set the LEDS to on or off.
 * @param yellowState 0 (off) or 1 (on)
 * @param greenState 0 (off) or 1 (on)
 * @param redState 0 (off) or 1 (on)
 * @returns void. Sets the pololu LED pins
 */
void setLEDs(int yellowState, int greenState, int redState) {
  ledYellow(yellowState);
  ledGreen(greenState);
  ledRed(redState);
}

/*
 * Abort if battery low
 * @returns void. Enter infinite loop and flash LEDS
 */
void checkBatteries()
{
  //avoid incorrect abort if robot is off and plugged in usb
  if (usbPowerPresent()) return;

  //if no USB check battery
  else
  {
    bool lowBattery = (readBatteryMillivolts() < 100) ? true : false;

    if (lowBattery)
      Serial.println("Low battery. Aborting");
    else if (!lowBattery)
      Serial.println("Batteries charged");

    //enter infite loop, delay is to take stress off cpu
    while (lowBattery)
    {
      setLEDs(1, 0, 1); delay(1000);
      setLEDs(0, 0, 1); delay(1000);
      setLEDs(1, 0, 0); delay(1000);
    }
  }
}
