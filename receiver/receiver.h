/** 
  Header file of encoder code
  Author  : - M. Luthfi Hariyadin
  Project : GPS Tracking
  For more information contact
  -> email: luthfihariyadin06@gmail.com
  Reference :
  -> https://www.benripley.com/diy/arduino/three-ways-to-read-a-pwm-signal-with-arduino/

**/

#ifndef RECEIVER_H
#define RECEIVER_H

class receiver
{
private:
    /* data */
    
public:
    receiver(byte pin);
    byte receiver_pin;
    void start(void(*userFunc)(void));
    void rising();
    void falling();
};


#endif
