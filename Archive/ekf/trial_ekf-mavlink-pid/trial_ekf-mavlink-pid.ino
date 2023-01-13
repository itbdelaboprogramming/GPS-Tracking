#include "motor.h"
#include "encoder.h"
#include "pidIr.h"
#include "LPF.h"

// Adding mavlink library
#include "mavlink_communication/mavlink/common/mavlink.h"
#include <TinyGPS++.h>

// ID System dan Component
#define MAVLINK_SYSTEM_ID 1 
#define MAVLINK_COMPONENT_ID 1

// Declare gps object from TinyGPS++ library
TinyGPSPlus gps;

#define LOOPTIME 10 //in ms

// To enter debug mode uncomment this line below. It will print the GPS data read from GPS.
//#define MONITOR_OMEGA
//#define DEBUG
//#define FILTER
//#define DEBUG_RECIVER
//#define DEBUG_ROTATE
//#define DEBUG_PWM
//#define DEBUG_GPS
#define DEBUG_ODOMETRY

// Pin untuk baca receiver adalah pin digital biasa
#define PIN_CH_1 48
#define PIN_CH_2 50
#define PIN_CH_3 52

// Nilai max pwm maju
#define MAX_PWM_MOVE 63 // Kecepatan max(255)/4
#define MAX_PWM_TURN 31 // Kecepatan max(255)/8
#define MAX_RPM_MOVE 191 // in RPM
#define MAX_RPM_TURN 90 // in RPM
#define PWM_THRESHOLD 100 // Batas diam motor 5/255
#define MAX_PWM 30

// Motor pin assignment
Motor motor_kiri(6,5,8); // Motor(int RPWM, int LPWM, int EN);
Motor motor_kanan(9,10,7);

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

pidIr pid_left_omega(0.5,2.0,0.016);//Kp = 0.5, Ki = 2.0, Kd = 0.016
pidIr pid_right_omega(0.5,2.0,0.016);

pidIr pid_left_pulse(0.05,0.0,16.0);
pidIr pid_right_pulse(0.05,0.0,16.0);

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

// EKF variable
volatile long left_velocity;
volatile long right_velocity;
volatile long forward_velocity;
volatile long turning_velocity;

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

// Define gpsData struct for handling the gps data.
struct gpsData {
  double latitude;
  double longitude;
  double heading;
  uint32_t satellites;
  float HDOP;
};

double K_Lat;
double K_Long;

// Declare gpsData struct
struct gpsData gpsData;

/** Timer **/ 
uint32_t tlast_heartbeat;
//uint32_t tlast_attitude;
//uint32_t tlast_gps;

/** Function to set execute at specific frequency **/
#define EXECUTE(TLAST, FREQ) if((curr_millis-TLAST) >= 1000/FREQ)
/** 
  @brief EXECUTE(TLAST, FREQ) will be execute at desired frequency
  @param TLAST the last time this function is executed in milisecond
  @param FREQ the frequency which this function will be executed in Hz
**/


void setup() {
  // put your setup code here, to run once:

  // Begin Serial comunication
  // Serial is for debugging and sending data to Computer
  // Serial1 is for receiving data from GPS Module
  // Serial2 is for Telemetry communication
  Serial.begin(9600);
  Serial.begin(9600);
  Serial2.begin(57600);

  K_Lat = 10000000.0;
  K_Long = 10000000.0;

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

  #ifdef DEBUG_PWM
  Serial.print("pwm_ki"); Serial.print("\t");
  Serial.print("pwm_ka"); Serial.print("\t");
  Serial.println();
  #endif

  #ifdef DEBUG_GPS
  Serial.print(("Latitude:")); Serial.print("\t");
  Serial.print(("Longitude:")); Serial.print("\t");
  Serial.print(("Heading:")); Serial.print("\t");
  Serial.print(("Num of Sat:")); Serial.print("\t");
  Serial.print(("HDOP:")); Serial.print("\t");
  Serial.println();
  #endif

  #ifdef DEBUG_ODOMETRY
  Serial.print(("Left_Velocity:")); Serial.print("\t");
  Serial.print(("Right_Velocity:")); Serial.print("\t");
  Serial.print(("Forward_Velocity:")); Serial.print("\t");
  Serial.print(("Turning_Velocity:")); Serial.print("\t");
  Serial.println();
  #endif
  
  /** Reset all timer to 0 **/
  //tlast_attitude = 0;
  //tlast_gps = 0;
  tlast_heartbeat = 0;
}

void loop() {
  // put your main code here, to run repeatedly:

  curr_millis = millis();   // Bookmark the time 

// Checking Serial1 port from the GPS Module
  //Serial.println(Serial1.available());
  if(Serial1.available()>0){
    // Checking if the NMEA data from Serial1 port is succesfully encoded by gps object.
    if(gps.encode(Serial1.read())){
        // Checking if location data from the GPS is in valid format
      if(gps.location.isValid()){
        // Assign all the GPS data read from the GPS module to gpsData struc

        gpsData.latitude = (gps.location.lat()*K_Lat);
        gpsData.longitude = (gps.location.lng()*K_Long);
        gpsData.heading = gps.course.deg();
        gpsData.satellites = gps.satellites.value();
        gpsData.HDOP = gps.hdop.hdop();
      }
    }
  }

  /** Sending heartbeat, attitude and gps data at 1 Hz **/
  EXECUTE(tlast_heartbeat, 1){
    send_heartbeat();
    //send_attitude(curr_millis, radians(45.00), radians(0.00), radians(30.00), radians(0.00), radians(0.00), radians(0.00));
    //send_gps(curr_millis, -68916370, 1076106390, 1000, 1000, 4500);
    send_gps(curr_millis, gpsData.latitude, gpsData.longitude, gpsData.heading, gpsData.satellites, gpsData.HDOP);
    tlast_heartbeat = curr_millis;
  }
  
  /** 
    Heartbeat is needed to informed the other that this device is exist and sending mavlink data
    send_attitude() and send_gps() are function that convert the data to format that can be sent using Mavlink {see below}
  **/

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
    left_omega = (curr_left_angle-prev_left_angle)/((curr_millis - prev_millis)/1000.0)/6.0; //in RPM
    right_omega = (curr_right_angle-prev_right_angle)/((curr_millis - prev_millis)/1000.0)/6.0; //in RPM    

    // Filtered
    filtered_left_omega = omega_left_lp.filter(left_omega, Ts/1000.0);
    filtered_right_omega = omega_right_lp.filter(right_omega, Ts/1000.0);
    filtered_ch1 = ch1_lp.filter(ch1, Ts/1000.0);
    filtered_ch2 = ch2_lp.filter(ch2, Ts/1000.0);

    // odometry input for EKF
    left_velocity = filtered_left_omega * 0.1; // in m/s
    right_velocity = filtered_right_omega * 0.1; // in m/s
    forward_velocity = (left_velocity + right_velocity)/2; // in m/s
    turning_velocity = (left_velocity - right_velocity)/0.325; // in rad/s
    
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
      /*
      if(curr_millis >= 5000){
        target_speed = 190.0*6.0/1000.0; //in deg/ms
        target_pulse = target_pulse + Ts*target_speed;
      }
      */
      //target_speed = 190.0*6.0/1000.0; //in deg/ms
      //target_pulse = target_pulse + Ts*target_speed;
      //target_speed_ki = 190.0*6.0/1000.0; //in deg/ms
      //target_pulse_ki = target_pulse_ki + Ts*target_speed_ki;
      
      // Compute pwm (uncomment what is needed)
      //pwm_ki = pid_left_angle.compute(target_angle,curr_left_angle,max_pwm,Ts);
      //pwm_ka = pid_right_angle.compute(target_angle,curr_right_angle,max_pwm,Ts);;
      
      //pwm_ki = pid_left_omega.compute(target_omega,filtered_left_omega,max_pwm,Ts);
      //pwm_ka = pid_right_omega.compute(target_omega,filtered_right_omega,max_pwm,Ts);

      //pwm_ki = pid_left_pulse.compute(target_pulse, curr_left_angle, MAX_PWM, Ts);
      //pwm_ka = pid_right_pulse.compute(target_pulse, curr_right_angle, MAX_PWM, Ts);
      
      pwm_ki = 25;
      pwm_ka = 25;
      
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
      //if (moveValue == 0){
        //target_pulse = 0;
      //} else {
        //target_pulse = target_pulse + Ts*target_speed;
      //}

      target_speed_ka = (moveValue + turnValue)*6.0/1000.0; //in deg/s
      target_speed_ki = (moveValue - turnValue)*6.0/1000.0; //in deg/s
      target_pulse_ka = target_pulse_ka + Ts*target_speed_ka;
      target_pulse_ki = target_pulse_ki + Ts*target_speed_ki;

      pwm_ka = pid_left_pulse.compute(target_pulse_ka, curr_left_angle, MAX_PWM, Ts);
      pwm_ki = pid_right_pulse.compute(target_pulse_ki, curr_right_angle, MAX_PWM, Ts);

      //pwm_ka = moveValue;
      //pwm_ki = moveValue;
      
      // Rotate motor
      motor_kiri.setEnable(pwm_ki);
      motor_kanan.setEnable(pwm_ka);
      
      motor_kiri.rotate(pwm_ki);
      motor_kanan.rotate(pwm_ka);
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
    Serial.print((target_speed_ka)*1000.0/6.0); Serial.print("\t");
    Serial.print((target_speed_ki)*1000.0/6.0); Serial.print("\t");
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

    #ifdef DEBUG_PWM
    Serial.print(pwm_ki); Serial.print("\t");
    Serial.print(pwm_ka); Serial.print("\t");
    Serial.println();
    #endif

    #ifdef DEBUG_GPS
    Serial.print(gpsData.latitude); Serial.print("\t");
    Serial.print(gpsData.longitude); Serial.print("\t");
    Serial.print(gps.course.deg()); Serial.print("\t");
    Serial.print(gps.satellites.value()); Serial.print("\t");
    Serial.print(gps.hdop.hdop()); Serial.print("\t");
    Serial.println();
    #endif

    #ifdef DEBUG_ODOMETRY
    Serial.print(left_velocity); Serial.print("\t");
    Serial.print(right_velocity); Serial.print("\t");
    Serial.print(forward_velocity); Serial.print("\t");
    Serial.print(turning_velocity); Serial.print("\t");
    Serial.println();
    #endif
  }
}

void send_heartbeat() {
    /** Message variable and buffer declaration for sending the heartbeat **/
    mavlink_message_t msg_send;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    /** 
      This function will fill the mavlink_message_t variable with the needed field 
      The message field for Heartbeat message can be seen at https://mavlink.io/en/messages/common.html#HEARTBEAT
    **/
    mavlink_msg_heartbeat_pack(
      MAVLINK_SYSTEM_ID,          /** System ID **/
      MAVLINK_COMPONENT_ID,       /** Component ID **/
      &msg_send,                  /** Message variable that will be filled **/
      /** Heartbeat Field **/
      10,                         /** Type of vehicle (use 10 for rover) {see -> https://mavlink.io/en/messages/common.html#MAV_TYPE}**/
      0,                          /** Type of Autopilot used (use 0 for generic autopilot) {see -> https://mavlink.io/en/messages/common.html#MAV_AUTOPILOT} **/
      0,                          /** Base system mode bitmap {see -> https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG} **/
      0,                          /** Custom mode (don't know yet what this value means, fill with 0) **/
      0                           /** System status of the vehicle (use 0 uninitialized system) {see -> https://mavlink.io/en/messages/common.html#MAV_STATE} **/
    );

    /** Continue to serialize the message sent to a buffer and send it through serial comunication **/
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_send);
    Serial2.write(buf,len);
};

void send_gps(
      uint32_t t,
      int32_t lat,                /** [degE7] Latitude **/
      int32_t lon,                /** [degE7] Longitude **/
      int32_t alt,                /** [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL. **/
      int32_t relative_alt,       /** [mm] Altitude above ground **/
      uint16_t hdg                /** [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX **/
  ){
    /** Message variable and buffer declaration for sending the GPS data **/
    mavlink_message_t msg_send;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    /** 
      This function will fill the mavlink_msg_global_position_int_pack variable with the needed field 
    **/
    mavlink_msg_global_position_int_pack(
        MAVLINK_SYSTEM_ID,        /** System ID **/
        MAVLINK_COMPONENT_ID,     /** Component ID **/
        &msg_send,                /** Message variable that will be filled **/
       /** GPS Field **/
        t,                        /** [ms] Timestamp (time since system boot). **/
        lat,                      /** [degE7] Latitude, expressed **/
        lon,                      /** [degE7] Longitude, expressed **/
        alt,                      /** [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL. **/
        relative_alt,             /** [mm] Altitude above ground **/
        0,                        /** [cm/s] Ground X Speed (Latitude, positive north) **/
        0,                        /** [cm/s] Ground Y Speed (Longitude, positive east) **/
        0,                        /** [cm/s] Ground Z Speed (Altitude, positive down) **/
        hdg                       /** [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX **/
     );
    
    /** Continue to serialize the message sent to a buffer and send it through serial comunication **/
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_send);
    Serial2.write(buf,len);
};

void send_attitude(
    uint32_t t,
    float roll,
    float pitch,
    float yaw,
    float rollspeed,
    float pitchspeed,
    float yawspeed
  ){
    /** Message variable and buffer declaration for sending the GPS data **/
    mavlink_message_t msg_send;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    /** 
      This function will fill the mavlink_msg_attitude_pack variable with the needed field 
    **/
     mavlink_msg_attitude_pack(
       MAVLINK_SYSTEM_ID,         /** System ID **/
       MAVLINK_COMPONENT_ID,      /** Component ID **/
       &msg_send,                 /** Message variable that will be filled **/
      /** Attitude Field **/
       t,                         /** [ms] Timestamp (time since system boot). **/
       roll,                      /** [radian] Roll **/
       pitch,                     /** [radian] Pitch **/
       yaw,                       /** [radian] Yaw **/
       rollspeed,                 /** [radian/s] Rollspeed **/
       pitchspeed,                /** [radian/s] Pitchspeed **/
       yawspeed                   /** [radian/s] Yawspeed **/
     );
    
    /** Continue to serialize the message sent to a buffer and send it through serial comunication **/
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_send);
    Serial2.write(buf,len);
};
