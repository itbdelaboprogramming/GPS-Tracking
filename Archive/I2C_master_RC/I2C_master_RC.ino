#include <Wire.h>
long t = 0;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  /**/
  if(millis()-t>=100){
    receiver();
    t = millis();
  }
  //receiver();
}

void receiver(){
  Wire.requestFrom(2, 4);
  byte a = Wire.read();
  byte b = Wire.read();
  byte c = Wire.read();
  byte d = Wire.read();
  Serial.print(a); Serial.print("\t");
  Serial.print(b); Serial.print("\t");
  Serial.print(c); Serial.print("\t");
  Serial.print(d); Serial.print("\t");
  Serial.println();
}
