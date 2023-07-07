/* RGB Common Cathode Test Program
	Dave Eslinger, 24 June, 2023
	GoSciTech Ardino Robotics Course 
  
  Note: On the Mega board, only pins 2-13 and 44-46 support the 
  analogWrite() function.  See language reference page for details:
  https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
*/

const byte RED_PIN = 45;
const byte GREEN_PIN = 46;
const byte BLUE_PIN = 44;
const int INTERVAL = 200;
int r, g, b;  // our color values, 0-255

void setup() {
  // put your setup code here, to run once:
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  r = 0;
  b = 0;
  g = 0;
}

void loop() {
  // Begin with full red only.
  g = 0;
  r = 255;
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
  delay(INTERVAL);

  b = 135;
  r = 0;
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
  delay(INTERVAL);

  b = 0;
  g = 255;
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
  delay(INTERVAL);





  // Now we are back to pure red, so repeat!
}