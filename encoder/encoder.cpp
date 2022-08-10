/** 
  Code to assign encoder pin in arduino
  Author  : Achmad Syahrul Irwansyah
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  -
**/

#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include "encoder.h"

Encoder::Encoder(int enc_a, int enc_b){
  pinMode(enc_a,INPUT_PULLUP);
  pinMode(enc_b,INPUT_PULLUP);

  //interrupt when changing
  attachPCINT(digitalPinToPCINT(enc_a), doEncoderA, CHANGE);
  attachPCINT(digitalPinToPCINT(enc_b), doEncoderB, CHANGE);

  enc_pos = 0;
};

void Encoder::doEncoderA(){
  // look for a low-to-high on channel A
  if (digitalRead(enc_a) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(enc_b) == LOW) {  
      enc_pos = enc_pos + 1;         // CW
    } 
    else {
      enc_pos = enc_pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(enc_b) == HIGH) {   
      enc_pos = enc_pos + 1;          // CW
    } 
    else {
      enc_pos = enc_pos - 1;          // CCW
    }
  }
}

void Encoder::doEncoderB(){  
  // look for a low-to-high on channel B
  if (digitalRead(enc_b) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(enc_a) == HIGH) {  
      enc_pos = enc_pos + 1;         // CW
    } 
    else {
      enc_pos = enc_pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel A to see which way encoder is turning  
    if (digitalRead(enc_a) == LOW) {   
      enc_pos = enc_pos + 1;          // CW
    } 
    else {
      enc_pos = enc_pos - 1;          // CCW
    }
  }
}
