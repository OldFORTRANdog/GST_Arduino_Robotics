/* B3_Omnispin.ino
This is a test sketch for autonomous driving mode of
the three-wheeled drive Bread Board Bot (BBbot, B^3)

Arduino: Arduino Mega 256 v3 Clone
Motor Shield: Adafruit assembled Motor Shield for Arduino v2
---->  http://www.adafruit.com/products/1438

Programmer: Dave Eslinger; 2019, July 7
*/

// NOT WORKING YET!!!!

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <math.h>
#include <BreadBoardBot.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Define Constants
// IO Pins used
const byte LEFT_BUMP_PIN = 47;    // Define DIGITAL Pins for left
const byte RIGHT_BUMP_PIN = 46;   // and right bump sensors

// Define 'ports' for motors.
const byte LEFT_MOTOR_PORT = 3;
const byte RIGHT_MOTOR_PORT = 1;
const byte BACK_MOTOR_PORT = 2;
// Create pointers to motor control objects
Adafruit_DCMotor *mLeft = AFMS.getMotor(LEFT_MOTOR_PORT);
Adafruit_DCMotor *mRight = AFMS.getMotor(RIGHT_MOTOR_PORT);
Adafruit_DCMotor *mBack = AFMS.getMotor(BACK_MOTOR_PORT);

// Define global variables
float direction;       // Velocity Vector Angle (DEGREES) from forward to drive. Positive is to the right = clockwise.
byte magnitude;       // Magnitude (0-255) of vector in given direction
long duration;        // Duration (in milliseconds)

byte mLeftdir;     // Clockwise or Counter clockwise for the 3 wheels
byte mBackdir;
byte mRightdir;

void setup(void) {
  Serial.begin(9600);  //Begin serial communcation
  AFMS.begin();  // create with the default frequency 1.6KHz
  // Turn off all motors
  mLeft->run(RELEASE);
  mBack->run(RELEASE);
  mRight->run(RELEASE);
  /*Set up Bump Pins with Arduino internal pullup resistors
  This will make them always high unless a bump switch is hit,
  which will make a connection to ground and they will read low. */
  pinMode(LEFT_BUMP_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUMP_PIN, INPUT_PULLUP);
  // Now wait for the RIGHT bumpsensor to be pressed
  while(digitalRead(RIGHT_BUMP_PIN)) {};
  while(digitalRead(RIGHT_BUMP_PIN)!= HIGH) {};
  delay(600); // Bump pin triggered and released, just give 0.2 seconds to get hands out of the way.
  Serial.println("Beginning");
}
void loop(void) {
  // Section for taking commands from Serial Input
  // N.B.  Need to comment out one bracket at end for the autonomous loop below
  /* if (Serial.available() > 0) { */
  /*   direction = Serial.parseInt(); */
  /*   magnitude = Serial.parseInt(); */
  /*   duration = Serial.parseInt(); */
  /* }  */

  // loop for driving an arbitrary pattern
  for ( byte leg = 1; leg < 7; leg++ ) {
    duration = 2000;              // Constants per leg: One second/leg
    magnitude = 150;              //                    50% max power
    bool brake = false;           //                    No braking
    switch (leg) {
      case 1: // Move forward 45 degrees right
      Serial.println("Forward 45 degrees right");
      direction = 45.;
      odrive(direction, magnitude, duration, brake, mLeft, mRight, mBack);
      break;
      case 2: // Move forward 45 degrees left
      Serial.println("Forward 45 degrees left");
      direction = -45.;
      odrive(direction, magnitude, duration, brake, mLeft, mRight, mBack);
      break;
      case 3: // Move backward
      Serial.println("Back");
      direction = 180.;
      odrive(direction, magnitude, duration, brake, mLeft, mRight, mBack);
      break;
      case 4: // Move left
      Serial.println("Left");
      direction = -90.;
      odrive(direction, magnitude, duration, brake, mLeft, mRight, mBack);
      break;
      case 5: // Spin clockwise for 2 seconds
      Serial.println("Spin clockwise for 2 seconds");
      magnitude = 120;
      duration = 2000;
      direction = 10;
      //brake = true; // hard stop
      otimedspin(direction, magnitude, duration, brake, mLeft, mRight, mBack);
      break;
      case 6: // Spin counterclockwise for 2 seconds
      Serial.println("Spin counterclockwise for 2 seconds");
      magnitude = 120;
      duration = 2000;
      direction = -10;
      //brake = true; // hard stop
      otimedspin(direction, magnitude, duration, brake, mLeft, mRight, mBack);
      break;
      default: // Stop and pause for 2 seconds
      Serial.println("DEFAULT: Stop for 2 seconds");
      magnitude = 0;
      duration = 2000;
      direction = 0;
      brake = true; // hard stop
      otimedspin(direction, magnitude, duration, brake, mLeft, mRight, mBack);
    }
    // odrive(magnitude, duration, brake, mLeft, mRight, mBack);

  }
  // Loop complete, so stop until LEFT bumper triggered and released, then rerun
  while(digitalRead(LEFT_BUMP_PIN)) {};  // Wait until pushed
  while(!digitalRead(LEFT_BUMP_PIN)) {}; // and released
  delay (600);                           // and 0.6 seconds to get out of the way
}
