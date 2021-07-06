/* B3_06_SpinSquare.ino
   Drive the TWO-WHEELED Bread Board Bot (BBbot, B^3)
   in a square, stopping at (or near!) end point.

   Arduino: Arduino Mega 256 v3 Clone
   Motor Shield: Adafruit assembled Motor Shield for Arduino v2
   ---->  http://www.adafruit.com/products/1438

   Programmer: Dave Eslinger; June 7, 2015
   Revisions:
*/
#include <Adafruit_MotorShield.h>
#include <BreadBoardBot.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Define Constants

// IO Pins used
const byte LEFT_BUMP_PIN = 47;    // Define DIGITAL Pins for left
const byte RIGHT_BUMP_PIN = 46;   // and right bump sensors

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
  for ( byte leg = 1; leg <= 4; leg++ ) {
    drive(SIDE_LENGTH, TESTSPEED, motorLeft, motorRight);// Forward for SIDE_LENGTH units
    spin(-90., TESTSPEED, motorLeft, motorRight); // 90 deg. to left
    allStop(FORWARD, motorLeft, motorRight);
    delay(500);  // Pause for 1/2 a second after turning
  }
  delay(3000);
  for ( byte leg = 1; leg <= 4; leg++ ) {
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
