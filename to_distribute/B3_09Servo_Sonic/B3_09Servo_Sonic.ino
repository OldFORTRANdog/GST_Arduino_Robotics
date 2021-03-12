/* B3_09_Servo_Sonic.ino

  JUST FOR TESTING sonar.ping_median

   Drive the TWO-WHEELED Bread Board Bot (BBbot, B^3)
   forward.   When within TARGET_DISTANCE_INCHES inches of something,
   turn servos to find clearest path and then go that way.
   When a whisker bump sensor on either side hits something,
   back up and turn slightly away from that direction and resume
   forward path.

   Stop at a specified distance from an object directly ahead.

   N.B.: When the pan servo is mounted with the control wire coming
   out toward the back of the robot, then 180 in servo coordinates is
   all the way LEFT and 0 is all the way RIGHT.  The B^3 coordinates
   are from -90 all the way left to 90, all the way right.  We need to
   use a map() function to correct for this.

   Arduino: Arduino Mega 256 v3 Clone
   Motor Shield: Adafruit assembled Motor Shield for Arduino v2
   ---->  http://www.adafruit.com/products/1438

   Programmer: Dave Eslinger; June 13, 2015
   Revisions:
	June 23, 2016	Added sonic sensors on servos. DLE
    June 21, 2018	Fixed coordinate issues with map() and
		added sonic.ping_median calls. DLE

*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <math.h>
#include <BreadBoardBot.h>
#include <NewPing.h>
//#ifdef NO_ECHO
//  #undef NO_ECHO
//  #define NO_ECHO 10000               // Value returned if there's no ping echo within the specified MAX_SENSOR_DISTANCE or max_cm_distance. Default=0
//#endif
#include <Servo.h>
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Define Servos
Servo panServo;
Servo tiltServo;

/* Define Constants */

// IO Pins used
const byte LEFT_BUMP_PIN = 47;    // Define DIGITAL Pins for left
const byte RIGHT_BUMP_PIN = 46;   // and right bump sensors
const byte SONIC_TRIGGER_PIN = 51;
const byte SONIC_ECHO_PIN = 50;

// Parameters controlling program behavior
// Bump behavior
const byte FORWARD_SPEED = 100;   // Define normal speeds
const byte BACKWARD_SPEED = 100;  // and backup/turn speed
const int  TURN_DURATION = 600;   // Turn length in milliseconds

// Sonic sensor
const float TARGET_DISTANCE_INCHES = 7;
const int MAX_SONIC_DISTANCE = 500;      // cm, optional, 500 cm is default

// Define 'ports' for motors.
const byte LEFT_MOTOR_PORT = 3;
const byte RIGHT_MOTOR_PORT = 1;
// Create pointers to motor control objects
Adafruit_DCMotor *motorLeft = AFMS.getMotor(LEFT_MOTOR_PORT);
Adafruit_DCMotor *motorRight = AFMS.getMotor(RIGHT_MOTOR_PORT);

/* Define new untrasonic sensor with trigger & echo pins and
   the maximum distance to be sensed. */
NewPing sonic(SONIC_TRIGGER_PIN, SONIC_ECHO_PIN, MAX_SONIC_DISTANCE);

/* Define servo pins */
const byte PANSERVOPIN = 10; // Servo 1 on AdaFruit Motor Shield
const byte TILTSERVOPIN = 9; // Servo 2

float pingDist; // define variable to use for distance measurements


void setup(void) {
  AFMS.begin();  // create with the default frequency 1.6KHz
  // Turn off all motors to start, just a good habit
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);

  panServo.attach(PANSERVOPIN);
  tiltServo.attach(TILTSERVOPIN);
  panServo.write(90);
  tiltServo.write(90);

  Serial.begin(9600);  //Begin serial communcation

  /*Set up Bump Pins with Arduino internal pullup resistors
    This will make them always high unless a bump switch is hit,
    which will make a connection to ground and they will read low. */
  pinMode(LEFT_BUMP_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUMP_PIN, INPUT_PULLUP);

  pinMode(SONIC_TRIGGER_PIN, OUTPUT);
  pinMode(SONIC_ECHO_PIN, INPUT);
  delay(2000);  // Two second delay to get the robot placed
}

void loop() {

  // Test some of the sonic library functions:
  Serial.print(sonic.ping_in());
  Serial.print(" inches, cm = ");
  Serial.print(sonic.ping_cm());
  Serial.print(", ping time (ms) = ");
  // int ping_milli = sonic.ping();
  int ping_milli = sonic.ping_median();
  Serial.print(ping_milli);
  Serial.print(", real inches = ");
  Serial.print(Distance_inches(ping_milli));
  Serial.print(", real cm = ");
  Serial.print(Distance_cm(ping_milli));
  Serial.print(", med ping = ");
  Serial.println(sonic.ping_median(7));
  Serial.print("NO_ECHO = ");
  Serial.println(NO_ECHO);

  //   Assuming no switches closed initially.  Drive forward:
  motorLeft->setSpeed(FORWARD_SPEED);
  motorRight->setSpeed(FORWARD_SPEED);
  pingDist = Distance_inches(sonic.ping_median());
  Serial.println("First straight distance = " + String(pingDist));
  while (digitalRead(LEFT_BUMP_PIN) && digitalRead(RIGHT_BUMP_PIN)
         && pingDist > TARGET_DISTANCE_INCHES) {
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
    pingDist = Distance_inches(sonic.ping_median());
    Serial.println("Straight distance = " + String(pingDist));
  }

  // If you got here, one of the bump switches was closed or B3 is too
  // close to something straight ahead

  // First check the LEFT sensor:
  if (! digitalRead(LEFT_BUMP_PIN)) { // the LEFT side switch was bumped
    motorLeft->setSpeed(BACKWARD_SPEED / 3); // Slowly back up and turn to right
    motorRight->setSpeed(BACKWARD_SPEED);
    motorLeft->run(BACKWARD);
    motorRight->run(BACKWARD);
    delay(TURN_DURATION);                  // for specified duration
    motorLeft->run(RELEASE);               // Then stop power to the motors
    motorRight->run(RELEASE);              // and move to next section of code
  }
  // Then check the right sensor:
  else if (! digitalRead(RIGHT_BUMP_PIN)) { // the RIGHT side switch was bumped
    motorLeft->setSpeed(BACKWARD_SPEED); // Slowly back up and turn to left
    motorRight->setSpeed(BACKWARD_SPEED / 3);
    motorLeft->run(BACKWARD);
    motorRight->run(BACKWARD);
    delay(TURN_DURATION);                 // for specified duration
    motorLeft->run(RELEASE);               // Then stop power to the motors
    motorRight->run(RELEASE);              // and move to next section of code
  }
  // It must have been the sonar sensor
  else {
    motorLeft->run(RELEASE);   // So stop power to the motors
    motorRight->run(RELEASE);  // and move to next section of code
    motorLeft->run(BACKWARD);   // So REVERSE power to the motors
    motorRight->run(BACKWARD);  // to BRIEFLY APPLY a brake
    delay(50);
    motorLeft->run(RELEASE);   // Then stop power to the motors
    motorRight->run(RELEASE);  // and move to next section of code
    float maxDist = -999.;
    int newDirection = 0;

    for (int i = 5; i <= 175; i += 5) {
      panServo.write(i);
      pingDist = Distance_inches(sonic.ping_median());
      if (pingDist > maxDist) {
        //newDirection = i - 90;
        newDirection = map(i, 180, 0, -90, 90);
        maxDist = pingDist;
        String outMsg = String("New maximum distance is " +
                               String(pingDist) + " at angle " + String(newDirection));
        Serial.println(outMsg);
      }
//      delay(20);
    }
//    for (int i = 175; i >= 5; i -= 5) {
//      panServo.write(i);
//      pingDist = Distance_inches(sonic.ping_median());
//      if (pingDist > maxDist) {
//        newDirection = map(i, 180, 0, -90, 90);
//        maxDist = pingDist;
//        String outMsg = String("New maximum distance is " +
//                               String(pingDist) + " at angle " + String(newDirection));
//        Serial.println(outMsg);
//      }
//      delay(20);
//    }
    //	Now turn to the new maximumum direction!
    panServo.write(90);
    spin(newDirection, 75, motorLeft, motorRight);
    spinStop(newDirection, motorLeft, motorRight); //Stops the spin
    delay(50);
  }

  /*That is all!  Now go back to the beginning of the loop and
     drive straight ahead until somehting is bumped. */
}

float Distance_inches(int ping) {
  float inches_per_sec = 13582.67;  // Equivilent to 345 m/s
  return ping * inches_per_sec * 1e-6 * 0.5;
}

float Distance_cm(int ping) {
  float cm_per_sec = 34500.;        // Equivilent to 345 m/s
  return ping * cm_per_sec * 1e-6 * 0.5;
}
