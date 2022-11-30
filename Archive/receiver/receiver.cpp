/** 
  Header file of encoder code
  Author  : - M. Luthfi Hariyadin
  Project : GPS Tracking
  For more information contact
  -> email: luthfihariyadin06@gmail.com
  Reference :
  -> https://www.benripley.com/diy/arduino/three-ways-to-read-a-pwm-signal-with-arduino/

**/

#include <PinChangeInterrupt.h>
#include "receiver.h"

receiver::receiver(byte pin){
    receiver_pin = pin;
    
}

void receiver::start(void(*userFunc)(void)){
    Serial.println("receiver mulai");

    attachPCINT(digitalPinToPCINT(receiver_pin), userFunc, RISING);
}

void receiver::rising(){
    Serial.println("RISE");
    attachPCINT(digitalPinToPCINT(receiver_pin), falling, FALLING);
}

void receiver::falling(){
    Serial.println("FALL");
    attachPCINT(digitalPinToPCINT(receiver_pin), rising, RISING);
}