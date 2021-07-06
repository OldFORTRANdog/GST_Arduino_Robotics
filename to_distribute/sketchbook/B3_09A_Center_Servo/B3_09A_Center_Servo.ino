
/* B3_09_Servo_center.ino

   Simply centers both the pan and the tilt servo at 90 degrees.
   This is used to help install them correctly.

   N.B.: When the pan servo is mounted with the control wire coming
   out toward the back of the robot, then 180 in servo coordinates is
   all the way LEFT and 0 is all the way RIGHT.  The B^3 coordinates
   are from -90 all the way left to 90, all the way right.  We need to
   use a map() function to correct for this.

   Arduino: Arduino Mega 256 v3 Clone
   Motor Shield: Adafruit assembled Motor Shield for Arduino v2
   ---->  http://www.adafruit.com/products/1438

   Programmer: Dave Eslinger; June 19, 2021
*/
#include <Adafruit_MotorShield.h>
#include <BreadBoardBot.h>
#include <Servo.h>            // New library to talk to servos

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Define Servos
Servo panServo;
Servo tiltServo;

/* Define Constants */

/* Define servo pins */
const byte PANSERVOPIN = 10; // Servo 1 on AdaFruit Motor Shield
const byte TILTSERVOPIN = 9; // Servo 2

void setup(void) {

  panServo.attach(PANSERVOPIN);
  tiltServo.attach(TILTSERVOPIN);
  panServo.write(90);
  tiltServo.write(90);
  delay(1000);
}

void loop() {
  panServo.write(45);
  delay(200);
  for (int i = 50; i <= 135.; i += 5) {
    panServo.write(i);
    delay(200);
  }

  panServo.write(90);
  delay(500);

  tiltServo.write(45);
  delay(200);
  for (int i = 50; i <= 135.; i += 5) {
    tiltServo.write(i);
    delay(200);
  }

  panServo.write(90);
  tiltServo.write(90);
  while (1) {};

}
