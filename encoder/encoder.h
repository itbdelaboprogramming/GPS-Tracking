/** 
  Code to assign encoder pin with arduino
  Author  : Achmad Syahrul Irwansyah
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  -
**/

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
    public:
    //contructor to assign encoder pin
    Encoder(int enc_a, int enc_b);

    void doEncoderA();
    void doEncoderB();
    
    int enc_a;
    int enc_b;

    int enc_pos;
};

#endif