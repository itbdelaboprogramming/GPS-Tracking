/** 
  Main code to rotate the motors with a certain target
  Author  : - Achmad Syahrul Irwansyah
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  - 
**/

#include "Motor/motor.h"
#include "encoder/encoder.h"
#include "pidIr/pidIr.h"

#define LOOPTIME 10 //in ms

// Motor pin assignment
Motor motor_kiri(9,10,13); // Motor(int RPWM, int LPWM, int EN);
Motor motor_kanan(6,5,14);

// Encoder pin assignment
Encoder enc_kiri(11,12); // Encoder(int pin_a, int pin_b);
Encoder enc_kanan(7,8);

// Encoder callback function
void callbackKiA(){encoder_kiri.doEncoderA();}
void callbackKiB(){encoder_kiri.doEncoderB();}
void callbackKaA(){encoder_kiri.doEncoderA();}
void callbackKaB(){encoder_kiri.doEncoderB();}

// Create PID object
pidIr pid_left_angle(0.275,0.055,0.0,LOOPTIME); // pidIr(float Kp, float Ki, float Kd, float Ts);
pidIr pid_right_angle(0.275,0.055,0.0,LOOPTIME);
pidIr pid_left_omega(0.275,0.055,0.0,LOOPTIME);
pidIr pid_right_omega(0.275,0.055,0.0,LOOPTIME);

// Timestamp variables
unsigned long curr_millis;
unsigned long prev_millis;

// Angle variables
volatile long curr_left_angle;
volatile long curr_right_angle;
volatile long prev_left_angle;
volatile long prev_right_angle;

// Omega variables
volatile long left_omega;
volatile long right_omega;

// Command variables
float target_angle = 90.0; // in deg
float target_omega = 30.0; // in RPM
int max_pwm = 200; // 8 bits digital (0-255)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Start to set the pin mode
  motor_kiri.start();
  motor_kanan.start();
  enc_kiri.start(callbackKiA, callbackKiB);
  enc_kanan.start(callbackKaA, callbackKaB);
}

void loop() {
  // put your main code here, to run repeatedly:

  curr_millis = millis();   // Bookmark the time 

  // Start timed loop for everything else (in ms)
  if (curr_millis - prev_millis >= 10) {
    // Set motor enable
    bool ena_ki = 1;
    bool ena_ka = 1;
    motor_kiri.setEnable(ena_ki);
    motor_kanan.setEnable(ena_ka);

    // Angle 
    curr_left_angle = enc_kiri.getPos()/532.0*360.0;  //in deg
    curr_right_angle = enc_kanan.getPos()/532.0*360.0;
         
    // Omega
    left_omega = (curr_left_angle-prev_left_angle)/((curr_millis - prev_millis)/1000.0)/6.0; //in RPM
    right_omega = (curr_right_angle-prev_right_angle)/((curr_millis - prev_millis)/1000.0)/6.0; //in RPM

    // Compute pwm (uncomment what is needed)
    int pwm_ki = pid_left_angle.compute(target_angle,curr_left_angle,max_pwm);
    int pwm_ka = pid_left_angle.compute(target_angle,curr_right_angle,max_pwm);;

    //int pwm_ki = pid_left_omega.compute(target_omega,left_omega,max_pwm);
    //int pwm_ka = pid_left_omega.compute(target_omega,right_omega,max_pwm);

    //int pwm_ki = 255;
    //int pwm_ka = 255;

    // Rotate motor
    motor_kiri.rotate(pwm_ki);
    motor_kanan.rotate(pwm_ka);

    // Saving the last value
    prev_right_angle = curr_right_angle;
    prev_left_angle = curr_left_angle;    
    prev_millis = curr_millis;   
  }
}