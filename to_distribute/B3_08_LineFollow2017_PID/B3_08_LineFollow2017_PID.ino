/* B3_LineFollow.ino
   Drive the TWO-WHEELED Bread Board Bot (BBbot, B^3)
   forward, following a black line, but pretty inefficiently.
   When a whisker bump sensor on either side hits something,
   back up and turn slightly away from that direction and resume
   forward path.
   Stop at a specified distance from an object directly ahead.
   Arduino: Arduino Mega 256 v3 Clone
   Motor Shield: Adafruit assembled Motor Shield for Arduino v2
   ---->  http://www.adafruit.com/products/1438
   Programmer: Dave Eslinger; June 13, 2015
   Revisions:
*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <math.h>
#include <BreadBoardBot.h>
#include <PID_v1.h>


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

/* Define Constants */

// IO Pins used
const byte LEFT_BUMP_PIN = 47;    // Define DIGITAL Pins for left
const byte RIGHT_BUMP_PIN = 46;   // and right bump sensors
const byte LEFT_IR_LED_PIN = 43;
const byte LEFT_IR_PIN = A11;
const byte RIGHT_IR_LED_PIN = 42;
const byte RIGHT_IR_PIN = A10;

// Parameters controlling program behavior
// Bump behavior
const byte FORWARD_SPEED = 120;   // Define normal speeds
const byte BACKWARD_SPEED = 100;  // and backup/turn speed
const int  TURN_DURATION = 600;   // Turn length in milliseconds

// Line Following
const byte FOLLOW_LEFT_SIDE = 1;
const float HI_LIGHT = 0.80;     // Keep the BRIGHT side of the line ABOVE this
// fraction of the Maximum read by that sensor
const float LO_LIGHT = 0.20;     // Keep the DARK side of the line BELOW this
// fraction of the Maximum read by that sensor

// Define 'ports' for motors.
const byte LEFT_MOTOR_PORT = 3;
const byte RIGHT_MOTOR_PORT = 1;
// Create pointers to motor control objects
Adafruit_DCMotor *motorLeft = AFMS.getMotor(LEFT_MOTOR_PORT);
Adafruit_DCMotor *motorRight = AFMS.getMotor(RIGHT_MOTOR_PORT);

/* Global Variables */
int leftIR;
int minLeftIR = 1026;
int maxLeftIR = -1024;
int rightIR;
int minRightIR = 1026;
int maxRightIR = -1024;
float brightLeft;
float darkRight;
float leftRange;
float rightRange;

void setup(void) {
  AFMS.begin();  // create with the default frequency 1.6KHz
  // Turn off all motors to start, just a good habit
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
  Serial.begin(9600);  //Begin serial communcation

  /*Set up Bump Pins with Arduino internal pullup resistors
    This will make them always high unless a bump switch is hit,
    which will make a connection to ground and they will read low. */
  pinMode(LEFT_BUMP_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUMP_PIN, INPUT_PULLUP);

  /* Define IR LED source pins as OUTPUTs and turn on the IR LEDs */
  pinMode(LEFT_IR_LED_PIN, OUTPUT);
  pinMode(RIGHT_IR_LED_PIN, OUTPUT);
  digitalWrite(LEFT_IR_LED_PIN, HIGH);
  digitalWrite(RIGHT_IR_LED_PIN, HIGH);

  /* Calibrate the IR sensors
     1) Place over the brightest target and hold down right bumper switch.
     2) Place over the darkest target and hold down right bumper again.
     3) All done!  The program
        calculates the 80% and 20% (or as set by HI_LIGHT% and LO_LIGHT% values)
        target values to keep the sensors at.
  */
  int lightNum = 0;
  while (digitalRead(RIGHT_BUMP_PIN) == HIGH) {} // Pause until the pin is grounded (our switch)
  delay(100);
  while (digitalRead(RIGHT_BUMP_PIN) == LOW) { // Get high light values until the pin
    // is no longer grounded
    leftIR = analogRead(LEFT_IR_PIN);
    rightIR = analogRead(RIGHT_IR_PIN);
    Serial.print(leftIR);
    Serial.print("  = leftIR, Calibrate LIGHT & Dark rightIR = ");
    Serial.println(rightIR);
    if ( leftIR > maxLeftIR ) {
      maxLeftIR = leftIR;
    }
    if ( leftIR < minLeftIR ) {
      minLeftIR = leftIR;
    }
    if ( rightIR > maxRightIR ) {
      maxRightIR = rightIR;
    }
    if ( rightIR < minRightIR ) {
      minRightIR = rightIR;
    }
  }
  leftRange = maxLeftIR - minLeftIR;
  rightRange = maxRightIR - minRightIR;
  brightLeft = maxLeftIR - ((1.0 - HI_LIGHT) * leftRange);
  darkRight = minRightIR + (LO_LIGHT * rightRange);

  while ( digitalRead(LEFT_BUMP_PIN) == HIGH) {} // Pause until you trigger the left bump switch
  delay(1000);                      // Then wait 1 second and start line following.
}

void loop() {

  leftIR = analogRead(LEFT_IR_PIN);
  rightIR = analogRead(RIGHT_IR_PIN);
  Serial.print(leftIR);           // We can only write ONE thing
  Serial.print(" = left, right = "); // at a time, therefore it takes
  Serial.println(rightIR);        // three statemens to get our output.

  if (leftIR >= brightLeft && rightIR <= darkRight) {
    motorLeft->setSpeed(FORWARD_SPEED);
    motorRight->setSpeed(FORWARD_SPEED);
  }
  else if (leftIR < brightLeft ) {
    motorLeft->setSpeed(20);       // Too far left, stop LEFT motor
    motorRight->setSpeed(FORWARD_SPEED);
  }
  else if (rightIR > darkRight) {
    motorRight->setSpeed(20);
    motorLeft->setSpeed(FORWARD_SPEED);

  }
  motorLeft->run(FORWARD);
  motorRight->run(FORWARD);
  delay(20);
}
