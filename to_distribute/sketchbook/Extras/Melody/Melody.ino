/*
  Melody

  Plays a melody

  circuit:
  - 8 ohm speaker on digital pin 31

  created 21 Jan 2010
  modified 30 Aug 2011
  by Tom Igoe

 
  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Tone

  Modified for GoSciTech Arduino Robotics on 7/9/2019 by Dave Eslinger
    Changed hard-coded pin 8 to TONE_PIN constant
    Modified 3/12/2021 Dave Eslinger, just change speaker pin
*/

#include "pitches.h"

// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

byte const TONE_PIN = 31;  // Change as needed

void setup() {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(TONE_PIN, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(TONE_PIN);
  }
}

void loop() {
  // no need to repeat the melody.
}
