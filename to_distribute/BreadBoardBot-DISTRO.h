/* Common Functions for Bread Board Bot

Arduino Robotics summer camp at GoSciTech 2015nn
SC Governor's School of Science and Math

Arduino: Arduino Mega 256 v3 Clone
Motor Shield: Adafruit assembled Motor Shield for Arduino v2
---->  http://www.adafruit.com/products/1438

Programmer & Instructor: Dave Eslinger; June 6, 2015
Major revisions:
July 3, 2015 DLE (changed motorshield pointer passing)
July 7, 2019 DLE Added omniwheel versions of drive (odrive), spin (ospin),
	and a timed spin (otimedspin)
July 22, 2023 DLE Corrected omniwheel programs, directionality and braking.

N.B.  All power/magnitudes are specified on scale 0-255, times used are all in microseconds,
	  and angles are in degrees where 0 is straight ahead and -90 to the left, +90 to the right.
*/

#include <Adafruit_MotorShield.h>

//=================================================================================
/* The units here determine units used everywhere for distance!
   Use either inches or centimeters, but remember that all subsequent calls to
   BreadBoardBot.h functions that use distanc will be in those units.
*/
const float trackWidth = 10.0; // DIstance between part of tire that is on the ground

const float DISTANCE_SLOPE = 0.1992;	 // From regression equation with data from DriveTest.ino
const float DISTANCE_INTERCEPT = +3.014; // Double check the sign!

const float ANGLE_SLOPE = 0.1992;	  // From regression equation with data from SpinTest.ino
const float ANGLE_INTERCEPT = +3.014; // Double check the sign!

//===============================================================================

void allStop(int oldDirection, Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight)
{
	mLeft->setSpeed(50);  // Note that we reset the speeds here; therefore,
	mRight->setSpeed(50); // we need to reset them in other routine.
	if (oldDirection == FORWARD)
	{
		mLeft->run(BACKWARD);
		mRight->run(BACKWARD);
	}
	else
	{
		mLeft->run(FORWARD);
		mRight->run(FORWARD);
	}
	delay(50);
	mLeft->run(RELEASE);
	mRight->run(RELEASE);
	return;
}

void spinStop(int angle, Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight)
{
	mLeft->setSpeed(50);  // Note that we reset the speeds here; therefore,
	mRight->setSpeed(50); // we need to reset them in other routine.
	if (angle > 0)
	{
		mLeft->run(BACKWARD);
		mRight->run(FORWARD);
	}
	else
	{
		mLeft->run(BACKWARD);
		mRight->run(FORWARD);
	}
	delay(50);
	mLeft->run(RELEASE);
	mRight->run(RELEASE);
	return;
}

float duration_per_distance(float distance, byte speed)
{
	/* Find drive time in milliseconds from relationship developed from
	observations of distance/time for a speed.  */
	float dist_per_sec = (DISTANCE_SLOPE * float(speed)) + DISTANCE_INTERCEPT; // in whatever units used, from data
	float duration = abs(distance) / dist_per_sec;							   // needed time in sec
	return duration * 1000.0;												   // Return in milliseconds
}

void drive(float distance, byte speed, Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight)
{
	byte direction;
	mLeft->setSpeed(speed); // Set both speeds
	mRight->setSpeed(speed);

	if (distance > 0)
	{ // Postive distance is forward
		direction = FORWARD;
	}
	else
	{
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

void spin(float degrees, byte speed, Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight)
{
	/* A spin turns moves the wheels in opposite directions.  Each
	needs to go the same distance, which is determined by the
	degrees parameter, around a circle with a diameter of the
	robot track width.

	The actual distance will be the Circumference * the fraction of the
	circle needed, given by the ratio degrees/360. */
	// float distance = PI * trackWidth * degrees / 360.;
	float distance = PI * trackWidth * degrees / 360.;
	float duration = duration_per_distance(distance, speed);
	mLeft->setSpeed(speed); // Set both speeds
	mRight->setSpeed(speed);
	if (degrees > 0)
	{
		/* Positive angle is spin turn to the right; therefore right motor goes
		backwards and left side goes forward */
		mRight->run(BACKWARD);
		mLeft->run(FORWARD);
	}
	else
	{ // Negative or 0 angle, turn to the left
		mRight->run(FORWARD);
		mLeft->run(BACKWARD);
	}
	delay(duration);
	mLeft->run(RELEASE);
	mRight->run(RELEASE);
	return;
}

void pivot(float degrees, byte speed, Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight)
{
	/* A pivot turns moves only one wheel, the one opposite the turn
	directions.  The needed distance is determined by the
	degrees parameter, around a circle with a RADIUS of the
	robot track width.

	The actual distance will be the Circumference * the fraction of the
	circle needed, given by the ratio degrees/360. */
	float distance = 2.0 * PI * trackWidth * degrees / 360.;
	float duration = duration_per_distance(distance, speed);
	mLeft->setSpeed(speed); // Set both speeds
	mRight->setSpeed(speed);
	if (degrees > 0)
	{
		/*		Positive angle is spin turn to the right; therefore right motor
		does nothing and left side goes forward
		*/
		mRight->run(RELEASE);
		mLeft->run(FORWARD);
	}
	else
	{ // Negative or 0 angle, turn to the left
		mRight->run(FORWARD);
		mLeft->run(RELEASE);
	}
	delay(duration);
	mLeft->run(RELEASE);
	mRight->run(RELEASE);
	return;
}
/* ========================= Omniwheel/holonomic functions ========================= */

float duration_per_angle(float angle, byte speed)
{
	/* Find drive time in milliseconds from relationship developed from
	observations of angle/time for a speed.  */
	float degree_per_sec = (ANGLE_SLOPE * float(speed)) + ANGLE_INTERCEPT; // in whatever units used, from data
	float duration = abs(angle) / degree_per_sec;						   // needed time in sec
	return duration * 1000.0;											   // Return in milliseconds
}

void odrive(float angle, byte magnitude, float duration, bool brake,
			Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight, Adafruit_DCMotor *mBack)
{
	// Define Trig onstants needed for mapping X,Y vectors to wheel coordinates
	const float cos30_or_sin60 = sin((M_PI * 60) / 180.); // cos(30 deg) = sin(60 deg)
	const float cos60_or_sin30 = 0.5; // cos(60 deg) = sin(30 deg)

	if (duration > 0)
	{
		Serial.println(String("\nIn odrive: \nangle = " + String(angle) +
							  ", magnitude = " + String(magnitude) +
							  ", duration = " + String(duration) +
							  " and brake = " + String(brake)));

		float xVector = magnitude * sin((M_PI * angle) / 180.);
		float yVector = magnitude * cos((M_PI * angle) / 180.);
		Serial.println(String("xVector, yVector = " + String(xVector) +
							  ", " + String(yVector)));

		// Find relative power needed for each wheel based on the target velocity vector
		/* N.B.: When back motor is mounted in standard position (with wires towards inside
			of chassis and connected to Motor connection 2), then its FORWARD direction
			is to the left, the opposite of our convention.  Therefore, all the xVector 
			components need a negative sign in front of them to correct the directionality.
			*/
		float backPower = -xVector; // Multiply by fudge factor to prevent rotation if needed
		float leftPower = -(cos60_or_sin30 * xVector) - (cos30_or_sin60 * yVector);
		float rightPower = -(cos60_or_sin30 * xVector) + (cos30_or_sin60 * yVector);
		Serial.println(String("backpower, leftpower, rightpower = " +
							  String(backPower) + ", " + String(leftPower) + ", " +
							  String(rightPower)));

		// Find the actual motor speeds, 0-255, needed.  N.B. still need motor direction!
		byte backSpeed = abs(backPower);
		byte leftSpeed =abs(leftPower);
		byte rightSpeed = abs(rightPower);

		// Set the speeds
		mBack->setSpeed(backSpeed);
		mLeft->setSpeed(leftSpeed);
		mRight->setSpeed(rightSpeed);

		/* Set Motor directions.  For Adafruit V2 Motorshield:
		1 is Clockwise (Positive motor direction, FORWARD)
		2 is Counterclockwise (Negative vector direction, BACKWARD)
		3 is Brake (Doesn't work at present)
		4 is Release = stop power, not driving, but not brake
		*/
		// We can use a trinary operator to set direction within run call

		mBack->run((backPower > 0) ? FORWARD : BACKWARD);
		mLeft->run((leftPower > 0) ? BACKWARD : FORWARD);
		mRight->run((rightPower > 0) ? FORWARD : BACKWARD);

		// Print out motor control details
		Serial.print("Speeds Back,Left,Right = ");
		Serial.print(copysign(backPower, backSpeed));
		Serial.print(", ");
		Serial.print(copysign(leftPower, leftSpeed));
		Serial.print(", ");
		Serial.println(copysign(rightPower, rightSpeed));

		// Run motors for the duration needed, converting from seconds to milliseconds
		delay(duration);

		// use fake brake if specified
		if (brake)
		{ // Not a real brake, but set power = 0, stop driving motors
			mBack->setSpeed(0);
			mLeft->setSpeed(0);
			mRight->setSpeed(0);
			mBack->run(RELEASE);
			mLeft->run(RELEASE);
			mRight->run(RELEASE);
		}
	}
	else
	{ // no duration entered, so stop all motors
		mBack->setSpeed(0);
		mLeft->setSpeed(0);
		mRight->setSpeed(0);
	}
}

void ospin(float angle, byte magnitude, bool brake,
		   Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight, Adafruit_DCMotor *mBack)
{
	/* Function omnispin spins the robot clockwise for a positive magnitude, and
	counterclockwise for a negative magnitude.
	*/
	long duration = duration_per_angle(angle, magnitude);

	if (duration > 0)
	{
		Serial.println(String("\nIn ospin: \nangle = " + String(angle) +
							  ", magnitude = " + String(magnitude) +
							  ", duration = " + String(duration) +
							  " and brake = " + String(brake)));

		byte backSpeed = abs(magnitude);
		byte leftSpeed = backSpeed;
		byte rightSpeed = backSpeed;

		// Set the speeds
		mBack->setSpeed(backSpeed);
		mLeft->setSpeed(leftSpeed);
		mRight->setSpeed(rightSpeed);

		// We can use a trinary operator to set direction within run call
		mLeft->run((angle > 0) ? BACKWARD : FORWARD);
		mRight->run((angle > 0) ? FORWARD : BACKWARD);
		mBack->run((angle > 0) ? BACKWARD : FORWARD);

		// Print out motor control details
		Serial.print("Speeds Back,Left,Right = ");
		Serial.println(backSpeed);

		// Run motors for the duration needed, duration is in milliseconds
		delay(duration);
		if (brake)
		{ // Not a real brake, but set power = 0, stop driving motors
			mBack->setSpeed(0);
			mLeft->setSpeed(0);
			mRight->setSpeed(0);
			mBack->run(RELEASE);
			mLeft->run(RELEASE);
			mRight->run(RELEASE);
		}
	} // end of duration > 0
	else
	{						// no duration entered, so stop all motors
		mBack->setSpeed(0); // set all speeds to 0 and
		mLeft->setSpeed(0);
		mRight->setSpeed(0);
		mBack->run(RELEASE); // stop driving the motors.
		mLeft->run(RELEASE);
		mRight->run(RELEASE);
		/* Note that this is closer to putting the bot in neutral than a real brake.
		 */
	}
}

void otimedspin(float direction, byte magnitude, float duration, bool brake,
				Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight, Adafruit_DCMotor *mBack)
{
	/* Function omnispin spins the robot clockwise for a positive magnitude, and
	counterclockwise for a negative magnitude.
	*/
	if (duration > 0)
	{
		Serial.print("direction = ");
		Serial.print(direction);
		Serial.print(", magnitude = ");
		Serial.print(magnitude);
		Serial.print(" and duration = ");
		Serial.println(duration);

		byte backSpeed = abs(magnitude);
		byte leftSpeed = backSpeed;
		byte rightSpeed = backSpeed;

		// Set the speeds
		mBack->setSpeed(backSpeed);
		mLeft->setSpeed(leftSpeed);
		mRight->setSpeed(rightSpeed);

		// We can use a trinary operator to set direction within run call
		mBack->run((direction > 0) ? FORWARD : BACKWARD);
		mLeft->run((direction > 0) ? FORWARD : BACKWARD);
		mRight->run((direction > 0) ? BACKWARD : FORWARD);

		// Print out motor control details
		Serial.print("Speeds Back,Left,Right = ");
		Serial.print(backSpeed);

		// Run motors for the duration needed, duration is in milliseconds
		delay(duration);

		if (brake)
		{ // Not a real brake, but set power = 0, stop driving motors
			mBack->setSpeed(0);
			mLeft->setSpeed(0);
			mRight->setSpeed(0);
			mBack->run(RELEASE);
			mLeft->run(RELEASE);
			mRight->run(RELEASE);
		}

	} // end of duration > 0
	else
	{						// no duration entered, so stop all motors
		mBack->setSpeed(0); // set all speeds to 0 and
		mLeft->setSpeed(0);
		mRight->setSpeed(0);
		mBack->run(RELEASE); // stop driving the motors.
		mLeft->run(RELEASE);
		mRight->run(RELEASE);
		/* Note that this is closer to putting the bot in neutral than a real brake.
		 */
	}
}
