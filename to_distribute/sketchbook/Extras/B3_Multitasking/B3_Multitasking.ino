/* B3_Multitasking.ino

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

/*  ======================= Define Constants, these cannot change ================= */

// IO Pins used
const int LEDPIN = LED_BUILTIN;  // the number of the LED pin

const byte LEFT_BUMP_PIN = 53;   // Define DIGITAL Pins for left
const byte RIGHT_BUMP_PIN = 52;  // and right bump sensors
const byte SONIC_TRIGGER_PIN = 49;
const byte SONIC_ECHO_PIN = 48;

// Parameters controlling program behavior
// Bump behavior
const byte FORWARD_SPEED = 100;   // Define normal speeds
const byte BACKWARD_SPEED = 100;  // and backup/turn speed
const int TURN_DURATION = 600;    // Turn length in milliseconds

// Sonic sensor
const float TARGET_DISTANCE_INCHES = 7;
const int MAX_SONIC_DISTANCE = 500;  // cm, optional, 500 cm is default

// Define 'ports' for motors.
const byte LEFT_MOTOR_PORT = 3;
const byte RIGHT_MOTOR_PORT = 1;

/* Define servo pins */
const byte PANSERVOPIN = 10;  // Servo 1 on AdaFruit Motor Shield

/*  Declare  and initialize GLOBAL Variables  */
unsigned long ledInc = 1000;  // Increment used to turn off and on the LED
unsigned long prevLEDMillis;  // Stores last LED update/loop time
unsigned long driveInc = 10;
unsigned long prevDriveMillis;
unsigned long servoInc = 750;
unsigned long prevServoMillis;

float pingDist;  // define variable to use for distance measurements

int angle = 90;     // Angle servo is pointing at, 0 to left, 180 to right, 90 directly forward.
int angleInc = 25;  // amount to change servo each time servoInc is exceeded

float maxDist = -999.;
int newDirection = 0;

byte ledState = HIGH;  // start with the LED turned on

/* ============ Create instances of motors and sonic sensor  ===========*/

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Define Servos
Servo panServo;

// Create pointers to motor control objects
Adafruit_DCMotor *motorLeft = AFMS.getMotor(LEFT_MOTOR_PORT);
Adafruit_DCMotor *motorRight = AFMS.getMotor(RIGHT_MOTOR_PORT);

// Define ultrasonic sensor with trigger & echo pins and
//   the maximum distance to be sensed.
NewPing sonic(SONIC_TRIGGER_PIN, SONIC_ECHO_PIN, MAX_SONIC_DISTANCE);

/* ======== Functions used in just this program =============== */

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
}

void loop() {

  unsigned long currentMillis = millis();  // get burrent time value in milliseconds.
//  Serial.println(String("currentMillis is " + String(currentMillis) + ", previous LED Millis is " + String(prevLEDMillis)));


  if (currentMillis - prevLEDMillis >= ledInc) {  // Check if LED increment has been exceeded. If so...
    prevLEDMillis = currentMillis;                // Update previous value then
    if (ledState == HIGH) {                       // Change ledState variable to other state (HIGH or LOW), and
      ledState = LOW;
    } else {
      ledState = HIGH;
    }
    digitalWrite(LEDPIN, ledState);  // Write out ledState to LEDPIN
  }                                  // end of LED section

  if (currentMillis - prevServoMillis >= servoInc) {  // Check if Servo increment has been exceeded. If so...
    prevServoMillis = currentMillis;                  // Update previous value then
    angle += angleInc;                                // Calculate new angle and
    if (angle <= 20 || angle >= 160) {                // Check to see if angle is at end of range if so
      angleInc = -angleInc;                           // Change increment to opposite direction
    }
    panServo.write(angle);
    // pingDist = Distance_inches(sonic.ping_median(5));
    pingDist = Distance_inches(sonic.ping());
    String outMsg = String("Ping distance is " + String(pingDist));
    Serial.println(outMsg);
    if (pingDist > maxDist) {
      newDirection = map(angle, 180, 0, -90, 90);
      maxDist = pingDist;
      String outMsg = String("New maximum distance is " + String(pingDist) + " at angle " + String(newDirection));
      Serial.println(outMsg);
    }
  }
}
