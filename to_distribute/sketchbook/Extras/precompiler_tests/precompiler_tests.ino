/* simple Arduino sketch to test precompiler options.

First version:  Dave Eslinger (DLE) July 7, 2019
      2024, July 7: Added Adafruit library to test redefining its options.
*/

#include <NewPing.h>


#define BACKPORT 2

#ifdef BACKPORT
#undef BACKPORT
#define BACKPORT 5
#else
#define  BACKPORT 123
#endif

//#ifdef BACKPORT
//#undef BACKPORT
// #define BACKPORT 4
//#endif

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  //Begin serial communcation
#if BACKPORT == 2
  Serial.println("It is 2!");
#elif BACKPORT == 4
  Serial.println("It is 4!");
#else
  Serial.print("Hmmm... BACKPORT value is ");
  Serial.println(BACKPORT);
#endif


#define NO_ECHO 500

// #ifndef NO_ECHO
// Serial.println("NO_ECHO is not to be defined");
// #elif NO_ECHO==0
// #undef NO_ECHO
// #define NO_ECHO 500
// Serial.println(String("NO_ECHO was 0, now it is " + String(NO_ECHO)));
// #else
// Serial.println(String("NO_ECHO is " + String(NO_ECHO)));
// #endif


Serial.println(String("NO_ECHO is " + String(NO_ECHO)));

}

void loop() {
  // put your main code here, to run repeatedly:
}
