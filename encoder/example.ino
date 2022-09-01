#include "encoder.h"

#define enc_a 7
#define enc_b 4

Encoder encoder_kiri(enc_a, enc_b); // Create an Encoder-object named encoder_kiri

// Function to recall the reading pulse function
void callbackA(){encoder_kiri.doEncoderA();}
void callbackB(){encoder_kiri.doEncoderB();}

int enc_a_data;
int enc_b_data;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  Serial.println("Testing encoder");
  Serial.println("starting test at ");
  Serial.print("Pin A : "); Serial.println(encoder_kiri.getPinA());
  Serial.print("Pin B : "); Serial.println(encoder_kiri.getPinB());
  encoder_kiri.start(callbackA,callbackB);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(encoder_kiri.getPos());
}