/** 
  Code to assign motor pin in arduino with BTS7960 motor driver
  Author  : Achmad Syahrul Irwansyah
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  -
**/

#include <Arduino.h>
#include "motor.h"

Motor::Motor(int _fwd, int _rev, int _ena){
    pinMode(_fwd,OUTPUT);
    pinMode(_rev,OUTPUT);
    pinMode(_ena,OUTPUT);

    ena = _ena;
    fwd = _fwd;
    rev = _rev;
};

void Motor::setEnable(bool en){
    if (en==1) {
        digitalWrite(ena,HIGH);
    } else {
        digitalWrite(ena,LOW);
    }
};

void Motor::rotate(int val){
    if (val >= 0) {
        analogWrite(fwd,val);
        analogWrite(rev,0);
    } else {
        analogWrite(fwd,0);
        analogWrite(rev,val);
    }
};