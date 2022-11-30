#include "motor.h"
#include "encoder.h"
#include "pidIr.h"
#include "LPF.h"

// Pin untuk baca receiver adalah pin digital biasa
#define PIN_CH_1 48
#define PIN_CH_2 50
#define PIN_CH_3 52

// Nilai max pwm maju
#define MAX_PWM_MOVE 63 // Kecepatan max(255)/4
#define MAX_PWM_TURN 31 // Kecepatan max(255)/8
#define SIGNAL_THRESHOLD 50 // Batas diam motor 5/255

// Nilai max rpm maju
#define MAX_RPM_MOVE 500 // Kecepatan max(255)/4
#define MAX_RPM_TURN 250 // Kecepatan max(255)/8

#define LOOPTIME 10 //in ms
//#define PI 3.14159
#define MAX_PWM 100

// To enter debug mode uncomment this line below. It will print the GPS data read from GPS.
//#define MONITOR_OMEGA
//#define DEBUG
//#define FILTER
#define DEBUG_ROTATE

// Motor pin assignment
Motor motor_kiri(5,6,10); // Motor(int RPWM, int LPWM, int EN);
//Motor motor_kanan(7,8,9);

// Encoder pin assignment (just use 2, 3, 10, 11, 12)
//Encoder enc_kiri(2,3); // Encoder(int pin_a, int pin_b);
//Encoder enc_kanan(11,12);
Encoder enc_kiri(11,12); // Encoder(int pin_a, int pin_b);
//Encoder enc_kanan(2,3);

// Encoder callback function
void callbackKiA(){enc_kiri.doEncoderA();}
void callbackKiB(){enc_kiri.doEncoderB();}
//void callbackKaA(){enc_kanan.doEncoderA();}
//void callbackKaB(){enc_kanan.doEncoderB();}

// Create PID object
pidIr pid_left_pulse(0.1,0.0,5);
//pidIr pid_right_pulse(0.5,0.0,0.0);
/* 
pidIr pid_left_angle(0.275,0.055,0.0,LOOPTIME); // pidIr(float Kp, float Ki, float Kd);
pidIr pid_right_angle(0.275,0.055,0.0,LOOPTIME);

pidIr pid_left_omega(0.5,2.0,0.016,LOOPTIME/1000.0);//Kp = 0.5, Ki = 2.0, Kd = 0.016
pidIr pid_right_omega(0.5,2.0,0.016,LOOPTIME/1000.0);
*/

// Create LPF object
/**/
float fc = 10; //fc = cut-off frequency (in Hz)
//float Ts = LOOPTIME/1000.0; //Ts = time sampling (in s)
LPF omega_left_lp(fc);
//LPF omega_right_lp(fc);

// Timestamp variables
unsigned long curr_millis;
unsigned long prev_millis;

// Pulse variables
volatile int32_t curr_left_pulse;
volatile int32_t curr_right_pulse;
volatile int32_t prev_left_pulse = 0;
volatile int32_t prev_right_pulse = 0;

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
int target_pwm = 0;

float target_speed = 0;
int32_t target_pulse = 0;
long time_start = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Start to set the pin mode
  motor_kiri.start();
  //motor_kanan.start();
  enc_kiri.start(callbackKiA, callbackKiB);
  //enc_kanan.start(callbackKaA, callbackKaB);

  // This will only run in debug mode
  #ifdef DEBUG
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
}

void loop() {
  // put your main code here, to run repeatedly:

  curr_millis = millis();   // Bookmark the time 

  // Start timed loop for everything else (in ms)
  if (curr_millis - prev_millis >= 10) {
    int Ts = curr_millis - prev_millis;
    float Ts_s = Ts * 0.001;

    // Pulse
    curr_left_pulse = enc_kiri.getPos();
    //curr_right_pulse = enc_kanan.getPos();
    float dPulseLeft = curr_left_pulse - prev_left_pulse;
    //float dPulseRight = curr_right_pulse - prev_right_pulse;

    // Angle 
    //curr_left_angle = enc_kiri.getPos()/532.0*360.0;  //in deg
    //curr_right_angle = enc_kanan.getPos()/532.0*360.0;
         
    // Omega
    //left_omega = (curr_left_angle-prev_left_angle)/((curr_millis - prev_millis)/1000.0)/6.0; //in RPM
    //right_omega = (curr_right_angle-prev_right_angle)/((curr_millis - prev_millis)/1000.0)/6.0; //in RPM

    // Filtered
    //filtered_left_omega = omega_left_lp.filter(left_omega);
    //filtered_right_omega = omega_right_lp.filter(right_omega);

    // Handling outliers
    /*
    if (filtered_left_omega < -1000.0 || filtered_right_omega < -1000.0) {
      filtered_left_omega = prev_left_omega;
      filtered_right_omega = prev_right_omega;
    }
    */

    // Print filter result
    #ifdef FILTER
    Serial.print(left_omega); Serial.print("\t");
    Serial.print(right_omega); Serial.print("\t");
    Serial.print(filtered_left_omega); Serial.print("\t");
    Serial.print(filtered_right_omega); Serial.print("\t");
    Serial.println();
    #endif

    // Compute pwm (uncomment what is needed)
    //int pwm_ki = pid_left_angle.compute(target_angle,curr_left_angle,max_pwm);
    //int pwm_ka = pid_right_angle.compute(target_angle,curr_right_angle,max_pwm);;

    //int pwm_ki = pid_left_omega.compute(target_omega,filtered_left_omega,max_pwm, Ts);
    //int pwm_ka = pid_right_omega.compute(target_omega,filtered_right_omega,max_pwm, Ts);

    //int pwm_ki = 0;
    //int pwm_ka = 0;
    int pwm_ki = 63;
    int pwm_ka = 0;

    if(curr_millis >= 5000){
      target_speed = 1.0;
      target_pulse=target_pulse + Ts*target_speed;
    }
    
    
    float speed_left = dPulseLeft/Ts * 1000;
    //float speed_right = dPulseRight/Ts * 1000;
    float rpm_left = dPulseLeft/Ts * 2.0 * PI * 1000;
    //float rpm_right = dPulseRight/Ts * 2.0 * PI * 1000;

    //int pwm_pid_left = pid_left_pulse.compute(target_speed, speed_left, MAX_PWM, Ts);
    //target_pwm = target_pwm + pid_left_pulse.compute(target_speed, speed_left, MAX_PWM, Ts);
    //target_pwm = constrain(target_pwm, 0, 63);
    target_pwm = pid_left_pulse.compute(target_pulse, curr_left_pulse, MAX_PWM, Ts);

    float filtered_speed_left = omega_left_lp.filter(speed_left, Ts_s);
    //float filtered_speed_right = omega_right_lp.filter(speed_right, Ts_s);

    // Rotate motor
    motor_kiri.setEnable(target_pwm);
    //motor_kanan.setEnable(pwm_ka);
    
    motor_kiri.rotate(target_pwm);
    //motor_kanan.rotate(pwm_ka);

    #ifdef DEBUG_ROTATE
    //Serial.print(curr_left_pulse); Serial.print("\t");
    //Serial.print(curr_right_pulse); Serial.print("\t");
    //Serial.print(target_pulse); Serial.print("\t");
    //Serial.print(curr_left_pulse); Serial.print("\t");
    //Serial.print(speed_right); Serial.print("\t");
    //Serial.print(rpm_left); Serial.print("\t");
    //Serial.print(rpm_right); Serial.print("\t");
    //Serial.print(filtered_speed_left); Serial.print("\t");
    //Serial.print(curr_millis);Serial.print("\t");
    Serial.print(target_pwm); Serial.print("\t");
    Serial.print(target_speed*1000); Serial.print("\t");
    Serial.print(speed_left);
    //Serial.print(PI,9); Serial.print("\t");
    Serial.println();
    #endif

    // Saving the last value
    //prev_right_angle = curr_right_angle;
    //prev_left_angle = curr_left_angle; 
    //prev_right_omega = filtered_right_omega;
    //prev_left_omega = filtered_left_omega;
    
    prev_millis = curr_millis;
    prev_left_pulse = curr_left_pulse;
    prev_right_pulse = curr_right_pulse;

    // Print result
    #ifdef DEBUG
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
