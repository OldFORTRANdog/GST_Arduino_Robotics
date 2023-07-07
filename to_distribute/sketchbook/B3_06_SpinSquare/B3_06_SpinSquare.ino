/* B3_06_SpinSquare.ino
   Drive the TWO-WHEELED Bread Board Bot (BBbot, B^3)
   in a square, stopping at (or near!) end point.

   Arduino: Arduino Mega 256 v3 Clone
   Motor Shield: Adafruit assembled Motor Shield for Arduino v2
   ---->  http://www.adafruit.com/products/1438

   Programmer: Dave Eslinger; June 7, 2015
   Revisions:
      2023, July 7: Changed bump pin assignments and loop index names. DLE
*/
#include <Adafruit_MotorShield.h>
#include <BreadBoardBot.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Define Constants

// IO Pins used
const byte LEFT_BUMP_PIN = 53;    // Define DIGITAL Pins for left
const byte RIGHT_BUMP_PIN = 52;   // and right bump sensors

const byte TESTSPEED = 155;
const float SIDE_LENGTH = 12; // the length of 1 side of the square

// Define 'ports' for motors.
const byte LEFT_MOTOR_PORT = 3;
const byte RIGHT_MOTOR_PORT = 1;
// Create pointers to motor control objects
Adafruit_DCMotor *motorLeft = AFMS.getMotor(LEFT_MOTOR_PORT);
Adafruit_DCMotor *motorRight = AFMS.getMotor(RIGHT_MOTOR_PORT);

void setup(void) {
  AFMS.begin();  // create with the default frequency 1.6KHz
  // Turn off all motors to start, just a good habit
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);

  // Now wait for the RIGHT bumpsensor to be pressed
  while (digitalRead(RIGHT_BUMP_PIN)) {};
  while (!digitalRead(RIGHT_BUMP_PIN)) {};
  delay(600); // Bump pin triggered and released, just give 0.6 seconds to get hands out of the way.
}

void loop(void) {

  // Autonomous loop for driving in a square
  for ( byte i = 1; i <= 4; i++ ) {
    drive(SIDE_LENGTH, TESTSPEED, motorLeft, motorRight);// Forward for SIDE_LENGTH units
    spin(-90., TESTSPEED, motorLeft, motorRight); // 90 deg. to left
    allStop(FORWARD, motorLeft, motorRight);
    delay(500);  // Pause for 1/2 a second after turning
  }
  delay(3000);
  for ( byte i = 1; i <= 4; i++ ) {
    /* Important! This "i" variable is completely different from the one used above. 
    Variables have a "scope" which is the range of the program over which they are valid.
    Each of these "i" variables is limited in scope to the loop in which it is defined.
    In general, all variables are limited to the part of the program in which they are 
    defined.  That is why we define our "global" variables and constants before any
    functions. They are then available in all following functions.
    */
    drive(SIDE_LENGTH, TESTSPEED, motorLeft, motorRight);// Forward for 10 inches
    spin(90., TESTSPEED, motorLeft, motorRight); // 90 deg. to left
    allStop(FORWARD, motorLeft, motorRight);
    delay(500);  // Pause for 1/2 a second after turning
  }

  // Both squares complete, so stop until LEFT bumper triggered and released, then rerun
  while (digitalRead(LEFT_BUMP_PIN)) {}; // Wait until pushed
  while (!digitalRead(LEFT_BUMP_PIN)) {}; // and released
  delay (600);                           // and 0.6 seconds to get out of the way
}
