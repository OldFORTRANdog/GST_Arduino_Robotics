/* RGB Common Cathode Demo Program
	Dave Eslinger, 19 July, 2016
	GoSciTech Ardino Robotics Course 
  
  Revisions:
     2023, July 7: Changed pin assignments. DLE

Note: On the Mega board, only pins 2-13 and 44-46 support the 
  analogWrite() function.  See language reference page for details:
  https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
*/

const byte RED_PIN = 45;
const byte GREEN_PIN = 46;
const byte BLUE_PIN = 44;
const int SPEED = 1;
int r, g, b; // our color values, 0-255

void setup() {
  // put your setup code here, to run once:
pinMode(RED_PIN, OUTPUT);
pinMode(GREEN_PIN, OUTPUT);
pinMode(BLUE_PIN, OUTPUT);
r = 255;
b = 0;
}

void loop() {
  // Begin with full red only.
	for (int i = 0; i <= 255; i++) { // Ramp UP Green
		g = i;
		analogWrite(RED_PIN, r);
		analogWrite(GREEN_PIN, g);
		analogWrite(BLUE_PIN, b);
		delay(SPEED);
	}
// Now it is Yellow
	for(int i = 255; i >= 0; i--) { // Ramp down Red
		r = i;
		analogWrite(RED_PIN, r);
		analogWrite(GREEN_PIN, g);
		analogWrite(BLUE_PIN, b);
		delay(SPEED);
	}
// Now it is pure Green
	for (int i = 0; i <=255; i++){ // Ramp up Blue
		b = i;
		analogWrite(RED_PIN, r);
		analogWrite(GREEN_PIN, g);
		analogWrite(BLUE_PIN, b);
		delay(SPEED);
	}
// Now it is Cyan
	for(int i = 255; i >= 0; i--) { // Ramp down Green
		g = i;
		analogWrite(RED_PIN, r);
		analogWrite(GREEN_PIN, g);
		analogWrite(BLUE_PIN, b);
		delay(SPEED);
	}
// Now it is pure Blue
	for (int i = 0; i <=255; i++){ // Ramp up Red
		r = i;
		analogWrite(RED_PIN, r);
		analogWrite(GREEN_PIN, g);
		analogWrite(BLUE_PIN, b);
		delay(SPEED);
	}
// Now it is Magenta
	for (int i = 255; i >= 0; i--) { // Ramp down Blue
		b = i;
		analogWrite(RED_PIN, r);
		analogWrite(GREEN_PIN, g);
		analogWrite(BLUE_PIN, b);
		delay(SPEED);
	}
// Now we are back to pure red, so repeat!	
}