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

// Pin untuk baca receiver adalah pin digital biasa
#define PIN_CH_1 48
#define PIN_CH_2 50
#define PIN_CH_3 52

// Nilai max pwm maju
#define MAX_PWM_MOVE 63 // Kecepatan max(255)/4
#define MAX_PWM_TURN 31 // Kecepatan max(255)/8
#define PWM_THRESHOLD 50 // Batas diam motor 5/255

// Motor pin assignment
Motor motor_kiri(5,6,10); // Motor(int RPWM, int LPWM, int EN);
Motor motor_kanan(7,8,9);

// Encoder pin assignment (just use 2, 3, 10, 11, 12)
Encoder enc_kiri(2,3); // Encoder(int pin_a, int pin_b);
Encoder enc_kanan(11,12);

// Encoder callback function
void callbackKiA(){enc_kiri.doEncoderA();}
void callbackKiB(){enc_kiri.doEncoderB();}
void callbackKaA(){enc_kanan.doEncoderA();}
void callbackKaB(){enc_kanan.doEncoderB();}

// Create PID object
pidIr pid_left_angle(0.275,0.055,0.0,LOOPTIME); // pidIr(float Kp, float Ki, float Kd, float Ts);
pidIr pid_right_angle(0.275,0.055,0.0,LOOPTIME);

pidIr pid_left_omega(0.5,2.0,0.016,LOOPTIME/1000.0);//Kp = 0.5, Ki = 2.0, Kd = 0.016
pidIr pid_right_omega(0.5,2.0,0.016,LOOPTIME/1000.0);

// Create LPF object
float fc = 3; //fc = cut-off frequency (in Hz)
float Ts = LOOPTIME/1000.0; //Ts = time sampling (in s)
LPF omega_left_lp(fc, Ts);
LPF omega_right_lp(fc, Ts);

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

// Command variables
float target_angle = 90.0; // in deg
float target_omega = 100.0; // in RPM
int max_pwm = 60; // 8 bits digital (0-255)

int pwm_ka;
int pwm_ki;

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
  Serial.print(F("Left_Omega:")); Serial.print("\t");
  Serial.print(F("Right_Omega:")); Serial.print("\t");
  Serial.print(F("Left_PWM:")); Serial.print("\t");
  Serial.print(F("Right_PWM:")); Serial.print("\t");
  Serial.print(F("Target_Omega:")); Serial.print("\t");
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
}

void loop() {
  // put your main code here, to run repeatedly:

  curr_millis = millis();   // Bookmark the time 

  // Start timed loop for everything else (in ms)
  if (curr_millis - prev_millis >= LOOPTIME) {

    // Menerima sinyal pwm dari receiver
    ch1 = pulseIn(PIN_CH_1, HIGH, 1000000/2);
    ch2 = pulseIn(PIN_CH_2, HIGH, 1000000/2);
    ch3 = pulseIn(PIN_CH_3, HIGH, 1000000/2);

    // Membatasi nilai pwm yang terbaca
    ch1 = constrain(ch1, 1000, 2000);
    ch2 = constrain(ch2, 1000, 2000);
    ch3 = constrain(ch3, 1000, 2000);

    // Angle 
    curr_left_angle = enc_kiri.getPos()/532.0*360.0;  //in deg
    curr_right_angle = enc_kanan.getPos()/532.0*360.0;
         
    // Omega
    left_omega = (curr_left_angle-prev_left_angle)/((curr_millis - prev_millis)/1000.0)/6.0; //in RPM
    right_omega = (curr_right_angle-prev_right_angle)/((curr_millis - prev_millis)/1000.0)/6.0; //in RPM

    // Filtered
    filtered_left_omega = omega_left_lp.filter(left_omega);
    filtered_right_omega = omega_right_lp.filter(right_omega);

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
      motor_kiri.setEnable(pwm_ka);
      motor_kanan.setEnable(pwm_ki);
      
      motor_kiri.rotate(pwm_ka);
      motor_kanan.rotate(pwm_ki);
    } else if(ch3 >= 1500){
      //Serial.println("Mode Auto");
      // Compute pwm (uncomment what is needed)
      //pwm_ki = pid_left_angle.compute(target_angle,curr_left_angle,max_pwm);
      //pwm_ka = pid_right_angle.compute(target_angle,curr_right_angle,max_pwm);;
      
      //pwm_ki = pid_left_omega.compute(target_omega,filtered_left_omega,max_pwm);
      //pwm_ka = pid_right_omega.compute(target_omega,filtered_right_omega,max_pwm);
      
      pwm_ki = 63;
      pwm_ka = 63;
      
      // Rotate motor
      motor_kiri.setEnable(pwm_ka);
      motor_kanan.setEnable(pwm_ki);
      
      motor_kiri.rotate(pwm_ka);
      motor_kanan.rotate(pwm_ki);
    } else {
      //Serial.println("Mode Manual");  
      if(ch1 >= 1500+PWM_THRESHOLD){
        moveValue = map(ch1, 1500+PWM_THRESHOLD, 2000, 0, MAX_PWM_MOVE);
      } else if(ch1 <= 1500-PWM_THRESHOLD){
        moveValue = map(ch1, 1500-PWM_THRESHOLD, 1000, 0, -MAX_PWM_MOVE);
      } else {
        moveValue = 0;
      }

      if(ch2 >= 1500+PWM_THRESHOLD){
        turnValue = map(ch2, 1500+PWM_THRESHOLD, 2000, 0, MAX_PWM_TURN);
      } else if(ch2 <= 1500-PWM_THRESHOLD){
        turnValue = map(ch2, 1500-PWM_THRESHOLD, 1000, 0, -MAX_PWM_TURN);
      } else {
        turnValue = 0;
      }

      pwm_ka = moveValue;
      pwm_ki = moveValue;
      
      // Rotate motor
      motor_kiri.setEnable(pwm_ka);
      motor_kanan.setEnable(pwm_ki);
      
      motor_kiri.rotate(pwm_ka);
      motor_kanan.rotate(pwm_ki);
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
    //Serial.print(enc_kiri.getPos()); Serial.print("\t");
    //Serial.print(enc_kanan.getPos()); Serial.print("\t");
    Serial.print(curr_left_angle); Serial.print("\t");
    Serial.print(curr_right_angle); Serial.print("\t");
    Serial.print(left_omega); Serial.print("\t");
    Serial.print(right_omega); Serial.print("\t");
    Serial.print(right_omega-left_omega); Serial.print("\t");
    Serial.print(target_angle); Serial.print("\t");
    Serial.print(target_omega); Serial.print("\t");
    Serial.print(pwm_ki); Serial.print("\t");
    Serial.print(pwm_ka); Serial.print("\t");
    Serial.println();
    #endif

    #ifdef MONITOR_OMEGA
    Serial.print(filtered_left_omega); Serial.print("\t");
    Serial.print(filtered_right_omega); Serial.print("\t");
    Serial.print(pwm_ki); Serial.print("\t");
    Serial.print(pwm_ka); Serial.print("\t");
    Serial.print(target_omega); Serial.print("\t");
    Serial.println();
    #endif
  }
}
