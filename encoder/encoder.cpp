/** 
  Code to assign encoder pin in arduino
  Author  : - Achmad Syahrul Irwansyah
            - M. Luthfi Hariyadin
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  - 
**/

#include <PinChangeInterrupt.h>
#include "encoder.h"

// Constructor to assign the encoder pins
Encoder::Encoder(int pin_a, int pin_b){
    enc_a = pin_a;
    enc_b = pin_b;
    enc_pos = 0;
};

// Method to start the encoder pulse reading by passing the doEncoder functions
void Encoder::start(void(*userFuncA)(void),void(*userFuncB)(void)){
    // Print information that encoder reading is starting
    Serial.println("Encoder Begin");

    // Set the pin as input pin Arduino
    pinMode(getPinA(), INPUT_PULLUP);
    pinMode(getPinB(), INPUT_PULLUP);
    
    // Interrupt when the encoder pulse changes, then do "userFunc" functions
    attachPCINT(digitalPinToPCINT(getPinA()), userFuncA, CHANGE);
    attachPCINT(digitalPinToPCINT(getPinB()), userFuncB, CHANGE);
}

/** Function to read the encoder pulse **/

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
