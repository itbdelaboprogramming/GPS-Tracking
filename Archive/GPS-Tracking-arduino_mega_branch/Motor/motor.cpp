/** 
  Code to assign motor pin in arduino with BTS7960 motor driver
  Author  : - Achmad Syahrul Irwansyah
            - M. Luthfi Hariyadin
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  -
**/

#include <Arduino.h>
#include "motor.h"

// Constructor to assign motor pin
Motor::Motor(int rpwm, int lpwm, int en){
  rPWM = rpwm;
  lPWM = lpwm;
  EN = en;
};

// Method to set the motor pin mode
void Motor::start(){
  pinMode(rPWM, OUTPUT);
  pinMode(lPWM, OUTPUT);
  pinMode(EN, OUTPUT);
}

// Method to set enable pin
// Set "ena" to be HIGH to activate the motor
void Motor::setEnable(bool ena){
    // Set the enable pin to be HIGH when "ena" is HIGH
    if (ena==1) {
        digitalWrite(EN,HIGH);
    } else {
        digitalWrite(EN,LOW);
    }
    Serial.print("Enable:");Serial.print("\t");
    Serial.println(ena);
};

// Method to assign PWM value and rotate the motor
void Motor::rotate(int val){
    // Rotate CW when the value of "val" is more than 0
    // and rotate CCW when the value is less then 0
    if (val >= 0) {
        analogWrite(rPWM, val);
        analogWrite(lPWM, 0);
    } else {
        analogWrite(rPWM, 0);
        analogWrite(lPWM, -val);
    }
    Serial.print("Value:");Serial.print("\t");
    Serial.println(val);
};

// Method to show pin assignment
void Motor::debug(){  
  Serial.print(rPWM);Serial.print("\t");
  Serial.print(lPWM);Serial.print("\t");
  Serial.print(EN);Serial.print("\t");
}
