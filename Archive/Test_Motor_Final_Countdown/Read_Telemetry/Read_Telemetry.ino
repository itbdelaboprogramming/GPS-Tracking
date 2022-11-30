// Adding mavlink library
//#include "mavlink_communication/mavlink/common/mavlink.h"

// ID System dan Component
#define MAVLINK_SYSTEM_ID 1 
#define MAVLINK_COMPONENT_ID 1

// To enter debug mode uncomment this line below. It will print the GPS data read from GPS.
#define MONITOR_OMEGA
//#define DEBUG
//#define FILTER
//#define DEBUG_RECIVER
//#define DEBUG_ROTATE

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

float target_speed = 0;
float target_pulse = 0;
float target_speed_ka = 0;
float target_pulse_ka = 0;
float target_speed_ki = 0;
float target_pulse_ki = 0;

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
  Serial1.begin(57600);

 // This will only run in debug mode
  #ifdef DEBUG
  //Serial.print(F("Left_Pulse:")); Serial.print("\t");
  //Serial.print(F("Right_Pulse:")); Serial.print("\t");
  Serial1.print(F("Left_Angle:")); Serial1.print("\t");
  Serial1.print(F("Right_Angle:")); Serial1.print("\t");
  Serial1.print(F("Left_Omega:")); Serial1.print("\t");
  Serial1.print(F("Right_Omega:")); Serial1.print("\t");
  Serial1.print(F("Target_Angle:")); Serial1.print("\t");
  Serial1.print(F("Target_Omega:")); Serial1.print("\t");
  Serial1.print(F("Left_PWM:")); Serial1.print("\t");
  Serial1.print(F("Right_PWM:")); Serial1.print("\t");
  Serial1.println();
  #endif 

  #ifdef MONITOR_OMEGA
  Serial1.print(("Left_Omega:")); Serial1.print("\t");
  Serial1.print(("Right_Omega:")); Serial1.print("\t");
  Serial1.print(("Left_PWM:")); Serial1.print("\t");
  Serial1.print(("Right_PWM:")); Serial1.print("\t");
  Serial1.print(("Target_Omega:")); Serial1.print("\t");
  Serial1.println();
  #endif

  #ifdef FILTER
  Serial1.print(F("Left_Omega:")); Serial1.print("\t");
  Serial1.print(F("Right_Omega:")); Serial1.print("\t");
  Serial1.print(F("Filtered_Left_Omega:")); Serial1.print("\t");
  Serial1.print(F("Filtered_Right_Omega:")); Serial1.print("\t");
  Serial1.println();
  #endif

  #ifdef DEBUG_RECIVER
  Serial1.print(F("Ch1:")); Serial1.print("\t");
  Serial1.print(F("Ch2:")); Serial1.print("\t");
  Serial1.print(F("Ch3:")); Serial1.print("\t");
  Serial1.println();
  #endif

  #ifdef DEBUG_ROTATE
  Serial1.print(F("PWM_Ki:")); Serial1.print("\t");
  Serial1.print(F("PWM_Ka:")); Serial1.print("\t");
  Serial1.print(F("Target:")); Serial1.print("\t");
  Serial1.print(F("Output_ki:")); Serial1.print("\t");
  Serial1.print(F("Output_ka:")); Serial1.print("\t");
  Serial1.println();
  #endif
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial1.available()>0){

    Serial.print(Serial1.read());
    
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
    Serial.print((target_speed_ka+target_speed_ki)*1000.0/6.0); Serial.print("\t");
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
