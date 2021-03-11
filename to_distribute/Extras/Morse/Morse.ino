/*
  Morse Code program by Dave Eslinger, based on the Blink program
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://www.arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */

int baseTime = 100;
String phrase = "  -.- ....- -.. .-.. .";
int onMult;
int offMult;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
	for (int i = 0; i <= phrase.length()-1; i++) {
		char signal = phrase[i];
		switch (signal) {
			case '.':
				onMult = 1;
				offMult = 1;
				break;
			case '-':
				onMult = 3;
				offMult = 1;
				break;
			case ' ':
				onMult = 0;
				offMult = 3;
				break;
		}
	  if (onMult > 0) {
		digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
		delay(baseTime*onMult);              // wait for a second
	  }
	  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
	  delay(baseTime*offMult);              // wait for a second		
	}
	delay(1000);
}
