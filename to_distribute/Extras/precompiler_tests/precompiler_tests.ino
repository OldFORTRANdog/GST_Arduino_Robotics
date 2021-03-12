/* simple Arduino sketch to test precompiler options.

First version:  Dave Eslinger (DLE) July 7, 2019

*/
#define BACKPORT 5

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  //Begin serial communcation
  #if BACKPORT == 2
  Serial.println("It is 2!");
  #elif BACKPORT == 4
  Serial.println("It is 4!");
  #else
  Serial.print("ERROR! Value is ");
  Serial.println(BACKPORT);
  #endif
}

void loop() {
  // put your main code here, to run repeatedly:

}
