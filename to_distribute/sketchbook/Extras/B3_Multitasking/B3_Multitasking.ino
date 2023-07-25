/* B3_Multitasking.ino

Omniwheel version of servo sonic with a blinking LED

Demonstration of using the micros() command and timers to perform multiple operations simultaneously.  
This would not be possible using delay(), since it stops almost all activity while waiting for the
delay to finish.

   Arduino: Arduino Mega 256 v3 Clone
   Motor Shield: Adafruit assembled Motor Shield for Arduino v2
   ---->  http://www.adafruit.com/products/1438

   Programmer: Dave Eslinger (DLE); July 10, 2023
   Revisions:

*/
#include <Adafruit_MotorShield.h>
#include <BreadBoardBot.h>
#include <Servo.h>
#include <NewPing.h>  // Another new library, this one to use the sonic sensor
// Option to change sonar behavior
// #define NO_ECHO 1450  // Changes value used in NewPing library when there is no sonar return.

bool TEST = false;
/*  ========================= Define Constants, these cannot change ========================= */

// IO Pins used
const int LEDPIN = LED_BUILTIN;  // the number of the LED pin

const byte LEFT_BUMP_PIN = 53;   // Define DIGITAL Pins for left
const byte RIGHT_BUMP_PIN = 52;  // and right bump sensors
const byte SONIC_TRIGGER_PIN = 39;
const byte SONIC_ECHO_PIN = 38;

// Parameters controlling program behavior
// Bump behavior
/* NOTE: This program uses the odrive functions in the BreadBoardBot library. Therefore, the
   speeds are given as range 0 to 255 and durations are in MICROSECONDS. */
const byte FORWARD_SPEED = 100;      // Define normal speeds,
const byte BACKWARD_SPEED = 70;     // and backup/turn speed
const int BACKWARD_DURATION = 250;  //backup length in Seconds
const int TURN_ANGLE = 40;           // Turn angle

// Sonic sensor
const float TARGET_DISTANCE_INCHES = 7.;
const int MAX_SONIC_DISTANCE = 500;  // Must be in cm, optional, 500 cm is default

// Define 'ports' for motors.
const byte LEFT_MOTOR_PORT = 3;
const byte RIGHT_MOTOR_PORT = 1;
const byte BACK_MOTOR_PORT = 2;

/* Define servo pins */
const byte PANSERVOPIN = 9;  // Servo 2 on AdaFruit Motor Shield
// const byte PANSERVOPIN = 10;  // Servo 1 on AdaFruit Motor Shield

/* ========================= Declare  and initialize GLOBAL Variables ========================= */
unsigned long ledInc = 1000;  // Increment used to turn off and on the LED
unsigned long prevLEDMillis;  // Stores last LED update/loop time
unsigned long driveInc = 10;
unsigned long prevDriveMillis;
unsigned long servoInc = 100;
unsigned long prevServoMillis;

float pingDist;  // define variable to use for distance measurements

int angle = 90;     // Angle servo is pointing at, 0 to right, 180 to left, 90 directly forward.
int angleInc = 25;  // amount to change servo each time servoInc is exceeded

float maxDist = -999.;  // Maximum distance seen with sonic sensor
int maxAngle = 90;      // Angle at which maximum distance is observed
int newDirection = 0;   // maxAngle mapped to robots coordinates --> 180 to 0 = -90 to +90

byte ledState = HIGH;  // start with the LED turned on

bool brake = true;  // Use brake after spin turns?

/* ========================= Create instances of motors and sonic sensor  ========================= */

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Define Servos
Servo panServo;

// Create pointers to motor control objects
Adafruit_DCMotor *motorLeft = AFMS.getMotor(LEFT_MOTOR_PORT);
Adafruit_DCMotor *motorRight = AFMS.getMotor(RIGHT_MOTOR_PORT);
Adafruit_DCMotor *motorBack = AFMS.getMotor(BACK_MOTOR_PORT);

// Define ultrasonic sensor with trigger & echo pins and
//   the maximum distance to be sensed.
NewPing sonic(SONIC_TRIGGER_PIN, SONIC_ECHO_PIN, MAX_SONIC_DISTANCE);

/* ========================= Functions used in just this program ========================= */

float Distance_inches(int ping) {
  float inches_per_sec = 13582.67;  // Equivilent to 345 m/s
  return ping * inches_per_sec * 1e-6 * 0.5;
}

float Distance_cm(int ping) {
  float cm_per_sec = 34500.;  // Equivilent to 345 m/s
  return ping * cm_per_sec * 1e-6 * 0.5;
}


void setup(void) {
  AFMS.begin();  // create with the default frequency 1.6KHz
  // Turn off all motors to start, just a good habit
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
  motorBack->run(RELEASE);

  panServo.attach(PANSERVOPIN);
  panServo.write(angle);

  Serial.begin(9600);  //Begin serial communcation

  /*Set up Bump Pins with Arduino internal pullup resistors
    This will make them always high unless a bump switch is hit,
    which will make a connection to ground and they will read low. */
  pinMode(LEFT_BUMP_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUMP_PIN, INPUT_PULLUP);

  pinMode(SONIC_TRIGGER_PIN, OUTPUT);
  pinMode(SONIC_ECHO_PIN, INPUT);

  pinMode(LEDPIN, OUTPUT);

  // Initialize Millis variables, should all be the same at the start

  prevLEDMillis = prevDriveMillis = prevServoMillis = millis();

  digitalWrite(LEDPIN, ledState);

  // Now wait for the RIGHT bumpsensor to be pressed
  while (digitalRead(RIGHT_BUMP_PIN)) {};
  while (!digitalRead(RIGHT_BUMP_PIN)) {};
  delay(300);  // Bump pin triggered and released, just give 0.3 seconds to get hands out of the way.
  if (TEST) {
    // motorLeft->setSpeed(FORWARD_SPEED);  
    // motorRight->setSpeed(FORWARD_SPEED);
    // motorBack->setSpeed(FORWARD_SPEED);  
    // motorLeft->run(FORWARD);
    // motorRight->run(FORWARD);
    // motorBack->run(FORWARD);
    odrive(-90, FORWARD_SPEED, BACKWARD_DURATION, true,
           motorLeft, motorRight, motorBack);
    Serial.println("\nospin\n");
    ospin(-90, FORWARD_SPEED, true,
              motorLeft, motorRight, motorBack);
    while (1) {};
  } else {
  }
}

void loop() {

  unsigned long currentMillis = millis();  // get current time value in milliseconds.
                                           // Serial.println(String("currentMillis is " + String(currentMillis)
                                           //                       + ", previous LED Millis is " + String(prevLEDMillis)));

  // Drive forward as default behavior
  motorLeft->setSpeed(FORWARD_SPEED);  // Slowly back up and turn to right
  motorRight->setSpeed(FORWARD_SPEED);
  motorLeft->run(FORWARD);
  motorRight->run(FORWARD);
  Serial.println("Flow test: normal drive");

  if (currentMillis - prevLEDMillis >= ledInc) {  // Check if LED increment has been exceeded. If so...
    prevLEDMillis = currentMillis;                // Update previous value then
    if (ledState == HIGH) {                       // Change ledState variable to other state (HIGH or LOW), and
      ledState = LOW;
    } else {
      ledState = HIGH;
    }
    digitalWrite(LEDPIN, ledState);  // Write out ledState to LEDPIN
    Serial.println("Flow test: BLINK LED");
  }  // end of LED section

  if (currentMillis - prevServoMillis >= servoInc) {  // Check if Servo increment has been exceeded. If so...
    prevServoMillis = currentMillis;                  // Update previous value then
    angle += angleInc;                                // Calculate new angle and
    if (angle <= 20 || angle >= 160) {                // Check to see if angle is at end of range if so
      angleInc = -angleInc;                           // Change increment to opposite
    }
    // panServo.write(angle);
    Serial.println("Flow test: PANNING SERVO to " + String(angle)
                   + ", distance is " + String(pingDist));
  }  // end of Servo section

  /* ========================= Get a new distance reading ========================= */
  int nping = 0;
  do {
    pingDist = Distance_inches(sonic.ping());
    nping++;
    currentMillis = millis();  // get current time value in milliseconds.
    Serial.println(String("Ping distance is " + String(pingDist)
                          + " and millis " + String(currentMillis)));
    if (pingDist == 0.0) {
      pingDist = MAX_SONIC_DISTANCE / 2.54;
      // Serial.println("Out of range, set pingDist to MAX_SONIC_DISTANCE in inches"
      // + String(MAX_SONIC_DISTANCE/2.54));
    }
  } while (pingDist <= 1.0);  // end of getting new ping section

  // Serial.println(String("Ping distance is " + String(pingDist) + " after "
  //                       + String(nping) + " pings at angle " + String(angle)
  //                       + " and millis " + String(currentMillis)));

  /* ========================= Check for new maximum pingDist ========================= */
  if (pingDist >= maxDist) {
    maxDist = pingDist;
    maxAngle = angle;
    // Serial.println(String("New maximum distance is " + String(pingDist)
    //                       + " at direction " + String(newDirection)));
  }

  /* ========================= Check the sonar sensor ========================= */
  if (pingDist <= TARGET_DISTANCE_INCHES) {
    newDirection = map(maxAngle, 180, 0, -90, 90);
    //  ospin(TURN_ANGLE, BACKWARD_SPEED, BACKWARD_DURATION, brake,
    //           motorLeft, motorRight, motorBack);
   Serial.println(String("Flow test: TOO close, " + String(pingDist)
                          + "inches, spin to " + String(newDirection)));

    // panServo.write(90);
    maxDist = -999.;
  }

  /* ========================= check the left sensor: ========================= */
  if (!digitalRead(LEFT_BUMP_PIN)) {  // the LEFT side switch was bumped
    Serial.println("Flow test: LEFT BUMP");
    odrive(180, -BACKWARD_SPEED, BACKWARD_DURATION, brake,
           motorLeft, motorRight, motorBack);
    ospin(TURN_ANGLE, BACKWARD_SPEED, brake,
              motorLeft, motorRight, motorBack);
  }

  /* ========================= check the right sensor: ========================= */
  if (!digitalRead(RIGHT_BUMP_PIN)) {  // the RIGHT side switch was bumped
    Serial.println("Flow test: RIGHT BUMP");
  }
}
