/* B3_10_HoloSquare.ino
   Drive in a square with the robot facing forward the entire time,
   using a loop for angle, and the omniwheel functions in BreadBoardBot.h
   library.

  Arduino: Arduino Mega 256 v3 Clone
  Motor Shield: Adafruit assembled Motor Shield for Arduino v2
  ---->  http://www.adafruit.com/products/1438

  Programmer: Dave Eslinger; December 2, 2014
  Revisions: 2015, May 25:   Changed for new motor configuration. DLE
			2015, June 12:  Changed into B3_ code style for GoSciTech 2015. DLE
			2015, July 9: Name change, cleaned up and additional comments added. DLE
			2015, July 10: Change default BACK motor port
			2019, July 5: Added the bump sensors to start each run. Also modified to
			 	use the new calls in BreadBoardBot.h library and saved as v2. DLE
      2021, July 6: Changed to use angle as loop variable, removed all old commented out code. DLE
*/
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
Adafruit_DCMotor *motorLeft = AFMS.getMotor(LEFT_MOTOR_PORT);
Adafruit_DCMotor *motorRight = AFMS.getMotor(RIGHT_MOTOR_PORT);
Adafruit_DCMotor *motorBack = AFMS.getMotor(BACK_MOTOR_PORT);

// Define global variables
float direction;       // Velocity Vector Angle (DEGREES) from forward to drive
float magnitude;       // Magnitude (0-100) of movement vector in given direction
float duration;        // Duration to drive at given velocity vector
bool brake;            // If true, power off motors and release.  Not a true brake.
 
byte motorLeftdir;     // Clockwise or Counter clockwise for the 3 wheels
byte motorBackdir;
byte motorRightdir;

void setup(void) {
  Serial.begin(9600);  //Begin serial communcation
  AFMS.begin();  // create with the default frequency 1.6KHz
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
  delay(600); // Bump pin triggered and released, just give 0.6 seconds to get hands out of the way.
  Serial.println("Beginning");
}
void loop(void) {

  /* Autonomous loop for driving in a square */
  duration = 1000;                          //  One second at each angle
  magnitude = 150;                          //  Set power
  brake = false;                       //  No braking

  // Loop for the polygon, a square in this case

  for ( int angle = 0; angle < 360; angle += 90 ) {   // Loop by 90 degrees = the number of sides/360.
    odrive(angle, magnitude, duration, brake,
           motorLeft, motorRight, motorBack);
  }
  // Stop at end of square by setting duration and magnitude to zero, and brake to true:
  duration = 0;
  magnitude = 10;
  brake = false;

  odrive(0, magnitude, duration, brake,   // Use 0 for angle, since the variable "angle" is not defined here
         motorLeft, motorRight, motorBack);

  /* Square complete and bot is stopped,
    so pause program until LEFT bumper triggered and released, then rerun
  */

  while (digitalRead(LEFT_BUMP_PIN)) {}; // Wait until pushed
  while (!digitalRead(LEFT_BUMP_PIN)) {}; // and released
  delay (600);                           // and 0.6 seconds to get out of the way
}
