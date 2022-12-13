/** 
  Code to run a rover with 3 modes: 
    1. Hold Mode is a mode to force stop the rover
    2. Manual Mode is a mode to run the rover with Remote-Control (RC)
    3. Auto Mode is still a blank code. It can be filled with anything. 
       For now, it is filled by PID tuning parameter code
  In this code, another method to control motor is provided. We try to control
  the motor using angle ramp-input instead of speed step-input.
  
  Maintainer  : Achmad Syahrul Irwansyah
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
**/

#include "motor.h"
#include "encoder.h"
#include "pidIr.h"
#include "LPF.h"

#define LOOPTIME 10 //in ms

// To enter debug mode uncomment this line below. It will print the GPS data read from GPS.
//#define MONITOR_OMEGA
#define DEBUG
//#define FILTER
//#define DEBUG_RECIVER
//#define DEBUG_ROTATE

// Pin untuk baca receiver adalah pin digital biasa
#define PIN_CH_1 50
#define PIN_CH_2 48
#define PIN_CH_3 52

// Nilai max pwm maju
#define MAX_PWM_MOVE 63 // Kecepatan max(255)/4
#define MAX_PWM_TURN 31 // Kecepatan max(255)/8
#define MAX_RPM_MOVE 50 // in RPM
#define MAX_RPM_TURN 25 // in RPM
#define PWM_THRESHOLD 150 // Batas diam motor 5/255
#define MAX_PWM 30

// Motor pin assignment
Motor motor_kanan(6,5,8); // Motor(int RPWM, int LPWM, int EN);
Motor motor_kiri(9,10,7);

// Encoder pin assignment (just use 2, 3, 10, 11, 12)
Encoder enc_kiri(3,2); // Encoder(int pin_a, int pin_b);
Encoder enc_kanan(12,11);

// Encoder callback function
void callbackKiA(){enc_kiri.doEncoderA();}
void callbackKiB(){enc_kiri.doEncoderB();}
void callbackKaA(){enc_kanan.doEncoderA();}
void callbackKaB(){enc_kanan.doEncoderB();}

// Create PID object
pidIr pid_left_angle(0.275,0.055,0.0); // pidIr(float Kp, float Ki, float Kd, float Ts);
pidIr pid_right_angle(0.275,0.055,0.0);

pidIr pid_left_omega(0.325,0.0012,0.0);//Kp = 0.325, Ki = 0.0008, Kd = 0.00
pidIr pid_right_omega(0.325,0.0012,0.0);

pidIr pid_left_pulse(0.05,0.0,0.8);
pidIr pid_right_pulse(0.05,0.0,0.0);

// Create LPF object
float fc = 3; //fc = cut-off frequency (in Hz)
LPF omega_left_lp(fc);
LPF omega_right_lp(fc);

float fc_receiver = 1; //fc_receiver = cut-off frequency in receiver (in Hz) 
LPF ch1_lp(fc_receiver);
LPF ch2_lp(fc_receiver);

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
volatile long prev_left_omega = 0;
volatile long prev_right_omega = 0;

// Filtered variables
volatile long filtered_left_omega;
volatile long filtered_right_omega;
volatile long filtered_ch1;
volatile long filtered_ch2;

// Command variables
float target_angle = 90.0; // in deg
float target_omega = 100.0; // in RPM
int max_pwm = 60; // 8 bits digital (0-255)

float target_speed = 0;
float target_pulse = 0;
float target_speed_ka = 0;
float target_pulse_ka = 0;
float target_speed_ki = 0;
float target_pulse_ki = 0;

int pwm_ka = 0;
int pwm_ki = 0;

// Receiver variables
int ch1; // output channel 1
int ch2; // output channel 2
int ch3; // output channel 3
int moveValue; // nilai pwm gerakan maju-mundur
int turnValue; // nilai pwm gerakan kiri-kanan

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Start to set the pin mode
  motor_kiri.start();
  motor_kanan.start();
  enc_kiri.start(callbackKiA, callbackKiB);
  enc_kanan.start(callbackKaA, callbackKaB);

  //Receiver pin
  pinMode(PIN_CH_1, INPUT);
  pinMode(PIN_CH_2, INPUT);
  pinMode(PIN_CH_3, INPUT);

  // This will only run in debug mode
  #ifdef DEBUG
  //Serial.print(F("Left_Pulse:")); Serial.print("\t");
  //Serial.print(F("Right_Pulse:")); Serial.print("\t");
  Serial.print(F("Left_Angle:")); Serial.print("\t");
  Serial.print(F("Right_Angle:")); Serial.print("\t");
  Serial.print(F("Left_Omega:")); Serial.print("\t");
  Serial.print(F("Right_Omega:")); Serial.print("\t");
  Serial.print(F("Target_Angle:")); Serial.print("\t");
  Serial.print(F("Target_Omega:")); Serial.print("\t");
  Serial.print(F("Left_PWM:")); Serial.print("\t");
  Serial.print(F("Right_PWM:")); Serial.print("\t");
  Serial.println();
  #endif 

  #ifdef MONITOR_OMEGA
  Serial.print(("Left_Omega:")); Serial.print("\t");
  Serial.print(("Right_Omega:")); Serial.print("\t");
  Serial.print(("Left_PWM:")); Serial.print("\t");
  Serial.print(("Right_PWM:")); Serial.print("\t");
  Serial.print(("Target_Omega_Kanan:")); Serial.print("\t");
  Serial.print(("Target_Omega_Kiri:")); Serial.print("\t");
  Serial.println();
  #endif

  #ifdef FILTER
  Serial.print(F("Left_Omega:")); Serial.print("\t");
  Serial.print(F("Right_Omega:")); Serial.print("\t");
  Serial.print(F("Filtered_Left_Omega:")); Serial.print("\t");
  Serial.print(F("Filtered_Right_Omega:")); Serial.print("\t");
  Serial.println();
  #endif

  #ifdef DEBUG_RECIVER
  Serial.print(F("Ch1:")); Serial.print("\t");
  Serial.print(F("Ch2:")); Serial.print("\t");
  Serial.print(F("Ch3:")); Serial.print("\t");
  Serial.println();
  #endif

  #ifdef DEBUG_ROTATE
  Serial.print(F("PWM_Ki:")); Serial.print("\t");
  Serial.print(F("PWM_Ka:")); Serial.print("\t");
  Serial.print(F("Target:")); Serial.print("\t");
  Serial.print(F("Output_ki:")); Serial.print("\t");
  Serial.print(F("Output_ka:")); Serial.print("\t");
  Serial.println();
  #endif

  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:

  curr_millis = millis();   // Bookmark the time 

  // Start timed loop for everything else (in ms)
  if (curr_millis - prev_millis >= 10) {
    float Ts = curr_millis - prev_millis;

    // Menerima sinyal pwm dari receiver
    ch1 = pulseIn(PIN_CH_1, HIGH, 1000000*2);
    ch2 = pulseIn(PIN_CH_2, HIGH, 1000000*2);
    ch3 = pulseIn(PIN_CH_3, HIGH, 1000000*2);

    // Membatasi nilai pwm yang terbaca
    ch1 = constrain(ch1, 1000, 2000);
    ch2 = constrain(ch2, 1000, 2000);
    ch3 = constrain(ch3, 1000, 2000);

    // Angle 
    curr_left_angle = enc_kiri.getPos()/532.0*360.0;  //in deg
    curr_right_angle = enc_kanan.getPos()/532.0*360.0;
         
    // Omega
    left_omega = (curr_left_angle-prev_left_angle)/(Ts/1000.0)/6.0; //in RPM
    right_omega = (curr_right_angle-prev_right_angle)/(Ts/1000.0)/6.0; //in RPM

    // Filtered
    filtered_left_omega = omega_left_lp.filter(left_omega, Ts/1000.0);
    filtered_right_omega = omega_right_lp.filter(right_omega, Ts/1000.0);
    filtered_ch1 = ch1_lp.filter(ch1, Ts/1000.0);
    filtered_ch2 = ch2_lp.filter(ch2, Ts/1000.0);

    // Print filter result
    #ifdef FILTER
    Serial.print(left_omega); Serial.print("\t");
    Serial.print(right_omega); Serial.print("\t");
    Serial.print(filtered_left_omega); Serial.print("\t");
    Serial.print(filtered_right_omega); Serial.print("\t");
    Serial.println();
    #endif

    //-------------------------------------Control Motor-----------------------------------------------//

    if(ch3 <= 1000){
      //Serial.println("Mode Hold");

      pwm_ki = 0;
      pwm_ka = 0;
      
      // Rotate motor
      motor_kiri.setEnable(pwm_ki);
      motor_kanan.setEnable(pwm_ka);
      
      motor_kiri.rotate(pwm_ki);
      motor_kanan.rotate(pwm_ka);
    } else if(ch3 >= 1500){
      //Serial.println("Mode Auto");
      
      //target_speed = 90.0*6.0/1000.0; //in deg/ms
      target_speed = 60.0; //in RPM
      target_pulse = target_pulse + Ts*target_speed;
      
      // Compute pwm (uncomment what is needed)
      //pwm_ki = pid_left_angle.compute(target_angle,curr_left_angle,max_pwm,Ts);
      //pwm_ka = pid_right_angle.compute(target_angle,curr_right_angle,max_pwm,Ts);;
      
      pwm_ki = pid_left_omega.compute(target_speed,filtered_left_omega,MAX_PWM,Ts);
      pwm_ka = pid_right_omega.compute(target_speed,filtered_right_omega,MAX_PWM,Ts);

      //pwm_ki = pid_left_pulse.compute(target_pulse, curr_left_angle, MAX_PWM, Ts);
      //pwm_ka = pid_right_pulse.compute(target_pulse, curr_right_angle, MAX_PWM, Ts);
      
      //pwm_ki = 30;
      //pwm_ka = 25;
      
      // Rotate motor
      motor_kiri.setEnable(pwm_ki);
      motor_kanan.setEnable(pwm_ka);
      
      motor_kiri.rotate(pwm_ki);
      motor_kanan.rotate(pwm_ka);
    } else {
      //Serial.println("Mode Manual");  
      if(filtered_ch1 >= 1500+PWM_THRESHOLD){
        moveValue = map(filtered_ch1, 1500+PWM_THRESHOLD, 2000, 0, MAX_RPM_MOVE);
      } else if(filtered_ch1 <= 1500-PWM_THRESHOLD){
        moveValue = map(filtered_ch1, 1500-PWM_THRESHOLD, 1000, 0, -MAX_RPM_MOVE);
      } else {
        moveValue = 0;
      }

      if(filtered_ch2 >= 1500+PWM_THRESHOLD){
        turnValue = map(filtered_ch2, 1500+PWM_THRESHOLD, 2000, 0, MAX_RPM_TURN);
      } else if(filtered_ch2 <= 1500-PWM_THRESHOLD){
        turnValue = map(filtered_ch2, 1500-PWM_THRESHOLD, 1000, 0, -MAX_RPM_TURN);
      } else {
        turnValue = 0;
      }

      target_speed = moveValue*6.0/1000.0; //in deg/s

      /*
      target_speed_ka = (moveValue + turnValue)*6.0/1000.0; //in deg/s
      target_speed_ki = (moveValue - turnValue)*6.0/1000.0; //in deg/s
      target_pulse_ka = target_pulse_ka + Ts*target_speed_ka;
      target_pulse_ki = target_pulse_ki + Ts*target_speed_ki;
      
      pwm_ki = pid_left_pulse.compute(target_pulse_ki, curr_left_angle, MAX_PWM, Ts);
      pwm_ka = pid_right_pulse.compute(target_pulse_ka, curr_right_angle, MAX_PWM, Ts);
      */

      target_speed_ki = moveValue + turnValue; //in RPM
      target_speed_ka = moveValue - turnValue;

      pwm_ki = pid_left_omega.compute(target_speed_ka,filtered_left_omega,max_pwm,Ts);
      pwm_ka = pid_right_omega.compute(target_speed_ki,filtered_right_omega,max_pwm,Ts);
      
      //pwm_ka = moveValue;
      //pwm_ki = moveValue;

      if (target_speed_ka == 0) {
        forceStop();
      } else {
        motor_kanan.setEnable(pwm_ka);
        motor_kanan.rotate(pwm_ka);
      }

      if (target_speed_ki == 0) {
        forceStop();
      } else {
        motor_kiri.setEnable(pwm_ki); 
        motor_kiri.rotate(pwm_ki);
      }
    }

    //------------------------------------------------------------------------------------------------//

    // Saving the last value
    prev_right_angle = curr_right_angle;
    prev_left_angle = curr_left_angle; 
    prev_right_omega = filtered_right_omega;
    prev_left_omega = filtered_left_omega; 
    prev_millis = curr_millis;   

    #ifdef DEBUG_RECIVER
    Serial.print(ch1); Serial.print("\t");
    Serial.print(ch2); Serial.print("\t");
    Serial.print(ch3); Serial.print("\t");
    Serial.println();
    #endif

    // Print result
    #ifdef DEBUG
    Serial.print(enc_kiri.getPos()); Serial.print("\t");
    Serial.print(enc_kanan.getPos()); Serial.print("\t");
    Serial.print(curr_left_angle); Serial.print("\t");
    Serial.print(curr_right_angle); Serial.print("\t");
    Serial.print(left_omega); Serial.print("\t");
    Serial.print(right_omega); Serial.print("\t");
    Serial.print(target_angle); Serial.print("\t");
    Serial.print(target_omega); Serial.print("\t");
    Serial.print(pwm_ki); Serial.print("\t");
    Serial.print(pwm_ka); Serial.print("\t");
    Serial.println();
    #endif

    #ifdef MONITOR_OMEGA
    Serial.print(filtered_left_omega); Serial.print("\t");
    Serial.print(filtered_right_omega); Serial.print("\t");
    //Serial.print(filtered_left_omega-filtered_right_omega); Serial.print("\t");
    Serial.print(pwm_ki); Serial.print("\t");
    Serial.print(pwm_ka); Serial.print("\t");
    Serial.print(target_speed_ka); Serial.print("\t");
    Serial.print(target_speed_ki); Serial.print("\t");
    Serial.println();
    #endif

    #ifdef DEBUG_ROTATE
    Serial.print(pwm_ki); Serial.print("\t");
    Serial.print(pwm_ka); Serial.print("\t");
    Serial.print(target_pulse); Serial.print("\t");
    Serial.print(curr_left_angle); Serial.print("\t");
    Serial.print(curr_right_angle); Serial.print("\t");
    Serial.println();
    #endif
  }
}

void forceStop () {
  pwm_ki = 0;
  pwm_ka = 0;
      
  // Rotate motor
  motor_kiri.setEnable(pwm_ki);
  motor_kanan.setEnable(pwm_ka);
      
  motor_kiri.rotate(pwm_ki);
  motor_kanan.rotate(pwm_ka);
}
