/* Timed Action example from http://playground.arduino.cc/Code/TimedAction#CurrentVersion */
#include <TimedAction.h>
// NOTE BENE: Be sure to change the #include "Wiring.h" statement to #include "Arduino.h" in line 33
// of TimedAction.h.  That makes it Arduino specific.
 
//pin / state variables
#define ledPin 13
boolean ledState = false;

// Define functions needed:
void blink(){
  ledState ? ledState=false : ledState=true;
  digitalWrite(ledPin,ledState);
} 

//this initializes a TimedAction class that will change the state of an LED every second.
TimedAction timedAction = TimedAction(1000,blink);
 
 
void setup(){
  pinMode(ledPin,OUTPUT);
  digitalWrite(ledPin,ledState);
}
 
void loop(){
  timedAction.check();
}
 
