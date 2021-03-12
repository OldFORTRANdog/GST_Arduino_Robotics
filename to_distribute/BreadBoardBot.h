/* Common Functions for Bread Board Bot

Arduino Robotics summer camp at GoSciTech 2015nn
SC Governor's School of Science and Math

Arduino: Arduino Mega 256 v3 Clone
Motor Shield: Adafruit assembled Motor Shield for Arduino v2
---->  http://www.adafruit.com/products/1438

Programmer & Instructor: Dave Eslinger; June 6, 2015
Major revisions:
July 3, 2015 DLE (changed motorshield pointer passing)
*/


#include <Adafruit_MotorShield.h>

#define TURNLEFT 0
#define TURNRIGHT 1

const float tireDiam = 2.75; // inches.  N.B. The units here determine units
const float trackWidth = 10.0; // inches      used everywhere for distance

void allStop(int oldDirection,
Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight) {
	mLeft->setSpeed(50);  // Note that we reset the speeds here; therefore,
	mRight->setSpeed(50); // we need to reset them in other routine.
	if (oldDirection == FORWARD) {
		mLeft->run(BACKWARD);
		mRight->run(BACKWARD);
	}
	else {
		mLeft->run(FORWARD);
		mRight->run(FORWARD);
	}
	delay(50);
	mLeft->run(RELEASE);
	mRight->run(RELEASE);
	return;
}

void spinStop(int angle,
Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight) {
	mLeft->setSpeed(50);  // Note that we reset the speeds here; therefore,
	mRight->setSpeed(50); // we need to reset them in other routine.
	if (angle > 0) {
		mLeft->run(BACKWARD);
		mRight->run(FORWARD);
	}
	else {
		mLeft->run(BACKWARD);
		mRight->run(FORWARD);
	}
	delay(50);
	mLeft->run(RELEASE);
	mRight->run(RELEASE);
	return;
}

float duration_per_distance( float distance, byte speed) {
	/* Find drive time in milliseconds from relationship developed from
	observations of distance/time for a speed.  */
	float dist_per_sec = (0.1294 * float(speed)) - 1.457;// in inches, from data
	float duration = abs(distance) / dist_per_sec;    // needed time in sec
	return duration * 1000.0 ;                        // Return in milliseconds
}

void drive(float distance, byte speed,
Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight) {
	byte direction;
	mLeft->setSpeed(speed);  // Set both speeds
	mRight->setSpeed(speed);

	if (distance > 0) {   // Postive distance is forward
		direction = FORWARD;
	}
	else {
		direction = BACKWARD;
	}
	float duration = duration_per_distance(distance, speed);
	/* Now move in the desired directions for that duration */
	mLeft->run(direction);
	mRight->run(direction);
	delay(duration);
	allStop(direction, mLeft, mRight);
	return;
}
void spin(float degrees, byte speed,
Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight) {
	/* A spin turns moves the wheels in opposite directions.  Each
	needs to go the same distance, which is determined by the
	degrees parameter, around a circle with a diameter of the
	robot track width.

	The actual distance will be the Circumference * the fraction of the
	circle needed, given by the ratio degrees/360. */
	// float distance = PI * trackWidth * degrees / 360.;
	float distance = PI * trackWidth * degrees / 360.;
	float duration = duration_per_distance(distance, speed);
	mLeft->setSpeed(speed);  // Set both speeds
	mRight->setSpeed(speed);
	if (degrees > 0) {
		/* Positive angle is spin turn to the right; therefore right motor goes
		backwards and left side goes forward */
		mRight->run(BACKWARD);
		mLeft->run(FORWARD);
	}
	else { // Negative or 0 angle, turn to the left
		mRight->run(FORWARD);
		mLeft->run(BACKWARD);
	}
	delay(duration);
	return;
}

void pivot(float degrees, byte speed,
Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight) {
	/* A pivot turns moves only one wheel, the one opposite the turn
	directions.  The needed distance is determined by the
	degrees parameter, around a circle with a RADIUS of the
	robot track width.

	The actual distance will be the Circumference * the fraction of the
	circle needed, given by the ratio degrees/360. */
	float distance = 2.0 * PI * trackWidth * degrees / 360.;
	float duration = duration_per_distance(distance, speed);
	mLeft->setSpeed(speed);  // Set both speeds
	mRight->setSpeed(speed);
	if (degrees > 0) {
/*		Positive angle is spin turn to the right; therefore right motor
		does nothing and left side goes forward
*/
		mRight->run(RELEASE);
		mLeft->run(FORWARD);
	}
	else { // Negative or 0 angle, turn to the left
		mRight->run(FORWARD);
		mLeft->run(RELEASE);
	}
	delay(duration);
	return;
}

void omnidrive(float direction, byte magnitude, float duration, bool brake,
Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight, Adafruit_DCMotor *mBack) {
	// Define Constants
	const float cos30sin60 = sqrt(3.0) / 2.0; // cos(30 deg) = sin(60 deg), need for wheel
	if ( duration > 0 ) {
		/* 		Serial.print("direction = ");
		Serial.print(direction);
		Serial.print(", magnitude = ");
		Serial.print(magnitude);
		Serial.print(" and duration = ");
		Serial.println(duration);
		*/
		if (brake) {            // Not a real brake, but set power = 0, stop driving motors
			mBack->setSpeed(0);
			mLeft->setSpeed(0);
			mRight->setSpeed(0);
			mBack-> run(RELEASE);
			mLeft-> run(RELEASE);
			mRight->run(RELEASE);
		}

		float xVector = magnitude * sin((M_PI * direction) / 180.);
		float yVector = magnitude * cos((M_PI * direction) / 180.);
		/*		Serial.print("xVector, yVector = ");
		Serial.print(xVector);
		Serial.print(", ");
		Serial.println(yVector);
		*/
		// Find relative power needed for each wheel based on the target velocity vector
		float backPower = xVector;  // Multiply by fudge factor to prevent rotation if needed
		float leftPower = 0.5 * xVector - cos30sin60 * yVector;
		float rightPower = 0.5 * xVector + cos30sin60 * yVector;

		// Find the actual motor speeds, 0-255, needed.  N.B. still need motor direction!
		byte backSpeed  = map(abs(backPower),  0, 100, 0, 255);
		byte leftSpeed  = map(abs(leftPower),  0, 100, 0, 255);
		byte rightSpeed = map(abs(rightPower), 0, 100, 0, 255);

		// Set the speeds
		mBack-> setSpeed(backSpeed);
		mLeft-> setSpeed(leftSpeed);
		mRight->setSpeed(rightSpeed);

		/* Set Motor directions.  For Adafruit V2 Motorshield:
		1 is Clockwise (Positive motor direction, FORWARD)
		2 is Counterclockwise (Negative vector direction, BACKWARD)
		3 is Brake (Doesn't work at present)
		4 is Release = stop power, not driving, but not brake

		We can use a trinary operator to set direction within run call

		mBack-> run((backPower  > 0) ? FORWARD : BACKWARD );
		mLeft-> run((leftPower  > 0) ? BACKWARD : FORWARD );
		mRight->run((rightPower > 0) ? FORWARD : BACKWARD );

		/*		//Print out motor control details
		Serial.print("Speeds Back,Left,Right = ");
		Serial.print(copysign(backPower, backSpeed));
		Serial.print(", ");
		Serial.print(copysign(leftPower, leftSpeed));
		Serial.print(", ");
		Serial.println(copysign(rightPower, rightSpeed));
		*/
		// Run motors for the duration needed, converting from seconds to milliseconds
		delay(duration);
	}
	else {                    // no duration entered, so stop all motors
		mBack->setSpeed(0);
		mLeft->setSpeed(0);
		mRight->setSpeed(0);
	}
}
