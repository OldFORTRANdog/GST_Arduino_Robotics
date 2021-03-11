/* RGB Common Cathode Demo Program
	Dave Eslinger, 19 July, 2016
	GoSciTech Ardino RObotics Course */
	
const byte RED_PIN = 7;
const byte GREEN_PIN = 6;
const byte BLUE_PIN = 5;

void setup() {
  // put your setup code here, to run once:
pinMode(RED_PIN, OUTPUT);
pinMode(GREEN_PIN, OUTPUT);
pinMode(BLUE_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
	for(int r = 0; r <= 1023; r++) {
		for (int g = 0; r <= 255; g+=10) {
			for (int b = 0; b <=255; b+=10){
				analogWrite(RED_PIN, r);
				analogWrite(GREEN_PIN, g);
				analogWrite(BLUE_PIN, b);
			}
		}
		delay(100);
	}
	
}