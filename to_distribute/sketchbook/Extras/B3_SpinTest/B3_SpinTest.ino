/* B3_SpinTest.ino
   Measure rotation rate of BreadBoardBot at different speeds.
   Used to calculate duration need for turn at a specific angle and speed,
   similar to what was done in DriveTest

  Arduino: Arduino Mega 256 v3 Clone
  Motor Shield: Adafruit assembled Motor Shield for Arduino v2
  ---->  http://www.adafruit.com/products/1438

  Programmer: Dave Eslinger; July 22, 2023

*/
#include <Adafruit_MotorShield.h>
#include <math.h>
#include <BreadBoardBot.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Define Constants
// IO Pins used
const byte LEFT_BUMP_PIN = 53;   // Define DIGITAL Pins for left
const byte RIGHT_BUMP_PIN = 52;  // and right bump sensors

// Define 'ports' for motors.
const byte LEFT_MOTOR_PORT = 3;
const byte RIGHT_MOTOR_PORT = 1;
const byte BACK_MOTOR_PORT = 2;
// Create pointers to motor control objects
Adafruit_DCMotor *motorLeft = AFMS.getMotor(LEFT_MOTOR_PORT);
Adafruit_DCMotor *motorRight = AFMS.getMotor(RIGHT_MOTOR_PORT);
Adafruit_DCMotor *motorBack = AFMS.getMotor(BACK_MOTOR_PORT);

// Define global variables
float direction;  // Positive for clockwise, negative for counter-clockwise
float magnitude;  // Magnitude (0-100) of movement vector in given direction
float duration;   // Duration to drive at given velocity vector
bool brake;       // If true, power off motors and release.  Not a true brake.

byte motorLeftdir;  // Clockwise or Counter clockwise for the 3 wheels
byte motorBackdir;
byte motorRightdir;

void setup(void) {
  Serial.begin(9600);  //Begin serial communcation
  AFMS.begin();        // create with the default frequency 1.6KHz
  // Turn off all motors
  motorLeft->run(RELEASE);
  motorBack->run(RELEASE);
  motorRight->run(RELEASE);
  /*Set up Bump Pins with Arduino internal pullup resistors
    This will make them always high unless a bump switch is hit,
    which will make a connection to ground and they will read low. */
  pinMode(LEFT_BUMP_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUMP_PIN, INPUT_PULLUP);
  // Now wait for the RIGHT bumpsensor to be pressed
  while (digitalRead(RIGHT_BUMP_PIN)) {};
  while (!digitalRead(RIGHT_BUMP_PIN)) {};
  delay(600);  // Bump pin triggered and released, just give 0.6 seconds to get hands out of the way.
  Serial.println("Beginning");
}

void loop(void) {
  /* Autonomous loop for driving in a square */
  duration = 1000;  //  One second at each angle
  brake = true;     //  No braking
  direction = +1;
  // Loop for power
  for (int power = 50; power <= 250; power += 50) {

    // Loop for iterations at given power
    for (int i = 1; i <= 3; i++) {
      otimedspin(direction, power, duration, brake,
            motorLeft, motorRight, motorBack);
      while (digitalRead(LEFT_BUMP_PIN)) {};
      while (!digitalRead(LEFT_BUMP_PIN)) {};
      delay(600);  // Bump pin triggered and released, just give 0.6 seconds to get hands out of the way.
    }

    while (digitalRead(RIGHT_BUMP_PIN)) {};
    while (!digitalRead(RIGHT_BUMP_PIN)) {};
    delay(600);  // Bump pin triggered and released, just give 0.6 seconds to get hands out of the way.
  }

    // Loop for iterations at given power
    for (int i = 1; i <= 3; i++) {
      otimedspin(direction, 255, duration, brake,
            motorLeft, motorRight, motorBack);
      while (digitalRead(LEFT_BUMP_PIN)) {};
      while (!digitalRead(LEFT_BUMP_PIN)) {};
      delay(600);  // Bump pin triggered and released, just give 0.6 seconds to get hands out of the way.
    }
}
