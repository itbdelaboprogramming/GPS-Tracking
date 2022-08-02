/** 
  Code to assign motor pin in arduino with BTS7960 motor driver
  Author  : Achmad Syahrul Irwansyah
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  -
**/

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor{
    public:
    Motor(int _fwd, int _rev, int _ena);

    void setEnable(bool en);
    void rotate(int val);

    int ena;
    int fwd;
    int rev;
};

#endif