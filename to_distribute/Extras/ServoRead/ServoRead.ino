/* Servo_Read.ino

   Read angle of two servo motors, panServo and TiltServo and
   print them out to the Serial Monitor.
   
   Used for debugging/calibrating Servo angles for BreadBoardBot

   Programmer: Dave Eslinger; July 26, 2016
   Revisions:  
*/
#include <Servo.h>

/* Define servo pins */
const byte PANSERVOPIN = 10; // Servo 1 on AdaFruit Motor Shield
const byte TILTSERVOPIN = 9; // Servo 2

/* Global Variables */
float panAngle, tiltAngle;

// Define Servos
Servo panServo;
Servo tiltServo;


void setup(void) {
  panServo.attach(PANSERVOPIN);
  tiltServo.attach(TILTSERVOPIN);
  // panServo.write(90);
  // tiltServo.write(90);

  Serial.begin(9600);  //Begin serial communcation

}

void loop() {
	for (int angle = 10; angle <= 170; angle+=5) {
	panServo.write(angle);
	tiltServo.write(angle);
	panAngle = panServo.read();
	tiltAngle = tiltServo.read();
	Serial.println(String("Pan is " + String(panAngle) + 
		", Tilt is " + String(tiltAngle)));
	delay(1000);
	
	}
}
