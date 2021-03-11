/* B3_TimedForward.ino
   Drive the TWO-WHEELED Bread Board Bot (BBbot, B^3)
   straight ahead for specified time and then stop
   
   Intially commented out code will impliment a a short 
   reverse brake on the motors. 

   Arduino: Arduino Mega 256 v3 Clone
   Motor Shield: Adafruit assembled Motor Shield for Arduino v2
   ---->  http://www.adafruit.com/products/1438

   Programmer: Dave Eslinger (DLE); June 3, 2015
   Major revisions:  
         6/7/2015, DLE: Cleaned it up for GoSciTech
         7/4/2015 DLE renamed

*/
#include <Wire.h>
#include <Adafruit_MotorShield.h> 
#include <math.h>
#include <TimedAction.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Define Constants
const byte TESTSPEED = 155;
const byte RED_PIN = 7;
const byte GREEN_PIN = 6;
const byte BLUE_PIN = 5;
const int SPEED = 4;
int r, g, b; // our color values, 0-255

// Define 'ports' for motors.
const byte LEFT_MOTOR_PORT = 3;
const byte RIGHT_MOTOR_PORT = 1;
// Create pointers to motor control objects
Adafruit_DCMotor *motorLeft = AFMS.getMotor(LEFT_MOTOR_PORT);
Adafruit_DCMotor *motorRight = AFMS.getMotor(RIGHT_MOTOR_PORT);

/* void Rainbow(void) {
  // Begin with full red only.
  for (int i = 0; i <= 255; i++) { // Ramp UP Green
    g = i;
    analogWrite(RED_PIN, r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN, b);
    delay(SPEED);
  }
  // delay(SPEED*  50);
  for (int i = 255; i >= 0; i--) { // Ramp down Red
    r = i;
    analogWrite(RED_PIN, r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN, b);
    delay(SPEED);
  }
  // Now it is pure Green
  for (int i = 0; i <= 255; i++) { // Ramp up Blue
    b = i;
    analogWrite(RED_PIN, r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN, b);
    delay(SPEED);
  }
  // Now it is Cyan
  for (int i = 255; i >= 0; i--) { // Ramp down Green
    g = i;
    analogWrite(RED_PIN, r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN, b);
    delay(SPEED);
  }
  // Now it is pure Blue
  for (int i = 0; i <= 255; i++) { // Ramp up Red
    r = i;
    analogWrite(RED_PIN, r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN, b);
    delay(SPEED);
  }
  // Now it is Magenta
  for (int i = 255; i >= 0; i--) { // Ramp down Blue
    b = i;
    analogWrite(RED_PIN, r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN, b);
    delay(SPEED);
  }
  // Now we are back to pure red, so repeat!
  // or turn them all off:
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN, 0);
    analogWrite(BLUE_PIN, 0);
}
*/

// TimedAction doRainbow = TimedAction(500, Rainbow);

void setup(void){
  AFMS.begin();  // create with the default frequency 1.6KHz
  // Turn off all motors
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
  Rainbow();
}
TimedAction doRainbow = TimedAction(500, Rainbow);

void loop(void){
  // Drive forward for 1 second at X/255 (~60% power)
  int duration = 1000; // in milliseconds
  byte speed = 255;
  motorLeft->setSpeed(speed);
  motorRight->setSpeed(speed);
  
  motorLeft->run(FORWARD);
  motorRight->run(FORWARD);
 doRainbow.check();
  delay(duration);
  
  motorLeft->run(RELEASE);   // Now stop the motors
  motorRight->run(RELEASE);  // However, this just coasts
/* Here is a braking method: run backwards a bit */
//  motorLeft->run(BACKWARD);  // Run backwards for a brief period
//  motorRight->run(BACKWARD);
//  delay(50);
//
//  motorLeft->run(RELEASE);     // and then stop the motors
//  motorRight->run(RELEASE);   
/* End of backwards section */
/* Another way to do the same braking technique, but with a
     function allStop: */
// allStop(FORWARD);

//  delay(2000);       // Pause for 2 seconds
while (1) {};      // What does this do?
}




//void allStop(int direction) {
//  motorLeft->setSpeed(100); // 100 on a 0-255 scale.
//  motorRight->setSpeed(100);
//  if (direction == FORWARD) {
//    motorLeft->run(BACKWARD);
//    motorRight->run(BACKWARD);
//  }
//  else {
//    motorLeft->run(BACKWARD);
//    motorRight->run(BACKWARD);
//  }
//  delay(50);
//  motorLeft->run(RELEASE);
//  motorRight->run(RELEASE);
//  return;
//}
void Rainbow(void) {
  // Begin with full red only.
  for (int i = 0; i <= 255; i++) { // Ramp UP Green
    g = i;
    analogWrite(RED_PIN, r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN, b);
    delay(SPEED);
  }
  // delay(SPEED*  50);
  for (int i = 255; i >= 0; i--) { // Ramp down Red
    r = i;
    analogWrite(RED_PIN, r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN, b);
    delay(SPEED);
  }
  // Now it is pure Green
  for (int i = 0; i <= 255; i++) { // Ramp up Blue
    b = i;
    analogWrite(RED_PIN, r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN, b);
    delay(SPEED);
  }
  // Now it is Cyan
  for (int i = 255; i >= 0; i--) { // Ramp down Green
    g = i;
    analogWrite(RED_PIN, r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN, b);
    delay(SPEED);
  }
  // Now it is pure Blue
  for (int i = 0; i <= 255; i++) { // Ramp up Red
    r = i;
    analogWrite(RED_PIN, r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN, b);
    delay(SPEED);
  }
  // Now it is Magenta
  for (int i = 255; i >= 0; i--) { // Ramp down Blue
    b = i;
    analogWrite(RED_PIN, r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN, b);
    delay(SPEED);
  }
  // Now we are back to pure red, so repeat!
  // or turn them all off:
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN, 0);
    analogWrite(BLUE_PIN, 0);
}

