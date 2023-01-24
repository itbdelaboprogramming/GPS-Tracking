#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>

#include "Motor.h"
#include "Encoder.h"
#include "LPF.h"
#include "pidIr.h"
#include <NewPing.h>

// For Debugging, uncomment one of these
//#define RECEIVER_RAW
//#define MOTOR_ANGLE_PULSE
//#define MOTOR_ANGLE_DEGREE
//#define MOTOR_ANGLE_RADIAN
//#define MOTOR_ANGLE_REVOLUTION
//#define MOTOR_SPEED_PPS
//#define MOTOR_SPEED_DPS
//#define MOTOR_SPEED_RPS
//#define MOTOR_SPEED_RPM
//#define TARGET_RPM
//#define PWM_RESPONSE
//#define VEHICLE_POSITION
//#define VEHICLE_SPEED
#define EKF_DATA


// Receiver PIN
#define PIN_CH_1 46
#define PIN_CH_2 48
#define PIN_CH_3 50
#define PIN_CH_4 52

// Motor PIN
#define RIGHT_MOTOR_REN_PIN 6
#define RIGHT_MOTOR_LEN_PIN 5
#define RIGHT_MOTOR_PWM_PIN 8

#define LEFT_MOTOR_REN_PIN 10
#define LEFT_MOTOR_LEN_PIN 9
#define LEFT_MOTOR_PWM_PIN 7

// Encoder PIN
#define RIGHT_ENC_PIN_A 12
#define RIGHT_ENC_PIN_B 11

#define LEFT_ENC_PIN_A 3
#define LEFT_ENC_PIN_B 2

// Infrared and Ultrasonic PIN
#define RIGHT_IR_PIN 24
#define LEFT_IR_PIN 22
#define ULTRASONIC_TRIGGER_PIN 26
#define ULTRASONIC_ECHO_PIN 28

// Constants
#define LOOP_TIME 10                // in milliseconds
#define PERIOD_TIME 2*pow(10,6)     // in microseconds
#define RECEIVER_CUT_OFF 1          // in Hertz (Hz)
#define ENCODER_CUT_OFF 3           // in Hertz (Hz)
#define ULTRASONIC_CUT_OFF 1        // in Hertz (Hz)
#define PWM_THRESHOLD 150           // in microseconds of receiver signal
#define MAX_RPM_MOVE 40             // in RPM for longitudinal movement
#define MAX_RPM_TURN 30             // in RPM for rotational movement
#define WHEEL_RADIUS 5.0            // in cm
#define WHEEL_DISTANCE 33.0         // in cm
#define MAX_DISTANCE 200            // in cm (maximum distance for ultrasonic)
#define LOWER_DISTANCE_BOUND 10     // in cm
#define UPPER_DISTANCE_BOUND 80     // in cm
#define MAX_PWM 60                  // saturation PWM for action control (0-255)

#define KP_RIGHT_MOTOR 0.325
#define KI_RIGHT_MOTOR 0.0012
#define KD_RIGHT_MOTOR 0.512

#define KP_LEFT_MOTOR 0.325
#define KI_LEFT_MOTOR 0.0012
#define KD_LEFT_MOTOR 0.512

Motor RightMotor(RIGHT_MOTOR_REN_PIN, RIGHT_MOTOR_LEN_PIN, RIGHT_MOTOR_PWM_PIN);
Motor LeftMotor(LEFT_MOTOR_REN_PIN, LEFT_MOTOR_LEN_PIN, LEFT_MOTOR_PWM_PIN);

Encoder RightEncoder(RIGHT_ENC_PIN_A, RIGHT_ENC_PIN_B);
Encoder LeftEncoder(LEFT_ENC_PIN_A, LEFT_ENC_PIN_B);

LPF Ch_1_lpf(RECEIVER_CUT_OFF);
LPF Ch_2_lpf(RECEIVER_CUT_OFF);

LPF RightRPM_lpf(ENCODER_CUT_OFF);
LPF LeftRPM_lpf(ENCODER_CUT_OFF);

LPF Ultrasonic_lpf(ULTRASONIC_CUT_OFF);

pidIr RightMotorPID(KP_RIGHT_MOTOR, KI_RIGHT_MOTOR, KD_RIGHT_MOTOR);
pidIr LeftMotorPID(KP_LEFT_MOTOR, KI_LEFT_MOTOR, KD_LEFT_MOTOR);

NewPing Sonar(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN, MAX_DISTANCE);

void callbackRA(){RightEncoder.doEncoderA();}
void callbackRB(){RightEncoder.doEncoderB();}
void callbackLA(){LeftEncoder.doEncoderA();}
void callbackLB(){LeftEncoder.doEncoderB();}

unsigned int ch_1_value;
unsigned int ch_2_value;
unsigned int ch_3_value;
unsigned int ch_4_value;

unsigned int ch_1_filtered;
unsigned int ch_2_filtered;

float right_rpm_filtered;
float left_rpm_filtered;

int move_value;
int turn_value;
float right_rpm_target = 0;
float left_rpm_target = 0;
int right_pwm = 0;
int left_pwm = 0;

// Vehicle Pose or State
float pose_x = 0;           // in cm
float pose_y = 0;           // in cm
float pose_theta = 0;       // in rad
float velocity_right = 0;   // in cm/s
float velocity_left = 0;    // in cm/s
bool in_calib_mode = false; 

int right_ir;
int left_ir;
float distance;
int data_id = 1;

unsigned long time_now = 0;
unsigned long time_last = 0;
unsigned long time_callib = 0;
float dt;

//ROS Communication
ros::NodeHandle nh;

geometry_msgs::Twist wheel_speed;
std_msgs::UInt8 ekf_state;

ros::Publisher wheel_speed_pub("wheel_speed", &wheel_speed);
ros::Publisher ekf_state_pub("ekf_state", &ekf_state);

void setup(){
    Serial.begin(9600);

    //Initiate ROS node
    nh.initNode();
    nh.advertise(wheel_speed_pub);
    nh.advertise(ekf_state_pub);
    
    RightMotor.begin();
    LeftMotor.begin();
    
    setupPinReceiver();

    RightEncoder.start(callbackRA, callbackRB);
    LeftEncoder.start(callbackLA, callbackLB);

    debugHeader();

    delay(2000);
}

void loop(){
    time_now = millis();
    if(time_now - time_last >= LOOP_TIME){
        dt = time_now - time_last;
        getReceiverSignal();

        ch_1_filtered = Ch_1_lpf.filter(ch_1_value, dt);
        ch_2_filtered = Ch_2_lpf.filter(ch_2_value, dt);

        //Calculate the robot position and velocity
        calculatePose();

        if(ch_4_value >= 1500 && ch_4_value <= 2000){
            // EKF Callibration
            in_calib_mode = true;
            calibMode();
        } else {
            in_calib_mode = false;
            data_id = 1;
            time_callib = 0;
        }

        if(ch_3_value <= 1250 && !in_calib_mode){
            // Mode HOLD
            vehicleStop();
            //vehicleGo(0, 0); //vehicleGo(pwm_right,pwm_left);
        } else if(ch_3_value >= 1750 && !in_calib_mode){
            // Mode AUTO
            //ultrasonicMode();
            ultrasonicGoForward();
        } else if(!in_calib_mode){
            // Mode MANUAL
            move_value = tuneReceiverSignaltoRPM(ch_1_filtered, MAX_RPM_MOVE);
            turn_value = tuneReceiverSignaltoRPM(ch_2_filtered, MAX_RPM_TURN);

            right_rpm_target = move_value + turn_value;
            left_rpm_target = move_value - turn_value;

            right_pwm = RightMotorPID.compute(right_rpm_target, right_rpm_filtered, MAX_PWM, dt);
            left_pwm = LeftMotorPID.compute(left_rpm_target, left_rpm_filtered, MAX_PWM, dt);

            if (right_rpm_target == 0 && left_rpm_target == 0){
                vehicleStop();
            } else {
                vehicleGo(right_pwm, left_pwm); 
            }
        }

        //Publish wheel_speed and ekf_state
        pubWheelSpeed();
        pubEKFState();
        
        time_last = time_now;
        debug();
    }
}

void setupPinReceiver(){
    pinMode(PIN_CH_1, INPUT);
    pinMode(PIN_CH_2, INPUT);
    pinMode(PIN_CH_3, INPUT);
    pinMode(PIN_CH_4, INPUT);
}

void getReceiverSignal(){
    ch_1_value = pulseIn(PIN_CH_1, HIGH, PERIOD_TIME);
    ch_2_value = pulseIn(PIN_CH_2, HIGH, PERIOD_TIME);
    ch_3_value = pulseIn(PIN_CH_3, HIGH, PERIOD_TIME);
    ch_4_value = pulseIn(PIN_CH_4, HIGH, PERIOD_TIME);

    ch_1_value = constrain(ch_1_value, 1000, 2000);
    ch_2_value = constrain(ch_2_value, 1000, 2000);
    ch_3_value = constrain(ch_3_value, 1000, 2000);
    ch_4_value = constrain(ch_4_value, 1000, 2000);
}

int tuneReceiverSignaltoRPM(int receiver_signal, int max_rpm){
    if(receiver_signal >= 1500 + PWM_THRESHOLD){
        return map(receiver_signal, 1500 + PWM_THRESHOLD, 2000, 0, max_rpm);
    } else if(receiver_signal <= 1500 - PWM_THRESHOLD){
        return map(receiver_signal, 1500 - PWM_THRESHOLD, 1000, 0, -max_rpm);
    } else {
        return 0;
    }
}

void vehicleStop(){
    RightMotor.stop();
    LeftMotor.stop();
    resetPID();
}

void vehicleGo(int right_pwm, int left_pwm){
    RightMotor.rotate(right_pwm);
    LeftMotor.rotate(left_pwm);
}

void resetPID(){
    RightMotorPID.reset();
    LeftMotorPID.reset();
}

void calculatePose(){
    float delta_angle_right = RightEncoder.getDeltaRad();
    float delta_angle_left = LeftEncoder.getDeltaRad();

    pose_x = pose_x + WHEEL_RADIUS/2.0 * (delta_angle_right + delta_angle_left) * sin(pose_theta);
    pose_y = pose_y + WHEEL_RADIUS/2.0 * (delta_angle_right + delta_angle_left) * cos(pose_theta);
    pose_theta = pose_theta + (delta_angle_right - delta_angle_left) * WHEEL_RADIUS/WHEEL_DISTANCE;

    pose_theta = wrapAngleFloatDegree(pose_theta);

    RightEncoder.measureOmega();
    LeftEncoder.measureOmega();

    right_rpm_filtered = RightRPM_lpf.filter(RightEncoder.getOmegaRPM(), dt);
    left_rpm_filtered = LeftRPM_lpf.filter(LeftEncoder.getOmegaRPM(), dt);

    velocity_right = right_rpm_filtered * PI/30.0 * WHEEL_RADIUS;
    velocity_left = left_rpm_filtered * PI/30.0 * WHEEL_RADIUS;
}

void ultrasonicMode(){
    right_ir = digitalRead(RIGHT_IR_PIN);
    left_ir = digitalRead(LEFT_IR_PIN);

    distance = Sonar.ping_cm();
    distance = Ultrasonic_lpf.filter(distance, dt);

    if(distance >= LOWER_DISTANCE_BOUND && distance <= UPPER_DISTANCE_BOUND){
        if(right_ir == 1 && left_ir == 0){
            ultrasonicTurnRight();
        } else if(right_ir == 0 && left_ir == 1){
            ultrasonicTurnLeft();
        } else {
            ultrasonicGoForward();
        }
    } else {
        vehicleStop();
    }
}

void ultrasonicTurnRight(){
    right_rpm_target = MAX_RPM_TURN;
    left_rpm_target = -MAX_RPM_TURN;

    right_pwm = RightMotorPID.compute(right_rpm_target, right_rpm_filtered, MAX_PWM, dt);
    left_pwm = LeftMotorPID.compute(left_rpm_target, left_rpm_filtered, MAX_PWM, dt);

    vehicleGo(right_pwm, left_pwm);
}

void ultrasonicTurnLeft(){
    right_rpm_target = -MAX_RPM_TURN;
    left_rpm_target = MAX_RPM_TURN;

    right_pwm = RightMotorPID.compute(right_rpm_target, right_rpm_filtered, MAX_PWM, dt);
    left_pwm = LeftMotorPID.compute(left_rpm_target, left_rpm_filtered, MAX_PWM, dt);

    vehicleGo(right_pwm, left_pwm);
}

void ultrasonicGoForward(){
    right_rpm_target = MAX_RPM_MOVE;
    left_rpm_target = MAX_RPM_MOVE;

    right_pwm = RightMotorPID.compute(right_rpm_target, right_rpm_filtered, MAX_PWM, dt);
    left_pwm = LeftMotorPID.compute(left_rpm_target, left_rpm_filtered, MAX_PWM, dt);

    vehicleGo(right_pwm, left_pwm);
}

void calibMode(){
    if(time_callib < 5000){
        time_callib += dt;
        data_id = 2;
        /*
        Serial.print(velocity_left); Serial.print(",");
        Serial.print(velocity_right); Serial.print(",");
        Serial.println(2);
        */
        vehicleStop();
    } else if(time_callib > 15000){
        time_callib += dt;
        vehicleStop();

        if(time_callib > 20000){
            data_id = 4;
            /*
            Serial.print(velocity_left); Serial.print(",");
            Serial.print(velocity_right); Serial.print(",");
            Serial.println(4);
            */
        } else {
            data_id = 3;
            /*
            Serial.print(velocity_left); Serial.print(",");
            Serial.print(velocity_right); Serial.print(",");
            Serial.println(3);
            */
        }
    } else {
        time_callib += dt;
        data_id = 0;
        /*
        Serial.print(velocity_left); Serial.print(",");
        Serial.print(velocity_right); Serial.print(",");
        Serial.println(0);
        */

        right_rpm_target = MAX_RPM_MOVE;
        left_rpm_target = MAX_RPM_MOVE;

        right_pwm = RightMotorPID.compute(right_rpm_target, right_rpm_filtered, MAX_PWM, dt);
        left_pwm = LeftMotorPID.compute(left_rpm_target, left_rpm_filtered, MAX_PWM, dt);

        vehicleGo(right_pwm, left_pwm);
    }
}

void pubWheelSpeed (){
  //Assign the wheel speed value
  wheel_speed.linear.x = velocity_right;
  wheel_speed.linear.y = velocity_left;

  //Publish the message
  wheel_speed_pub.publish(&wheel_speed);
}

void pubEKFState (){
  //Assign the EKF state value
  ekf_state.data = data_id;

  //Publish the message
  ekf_state_pub.publish(&ekf_state);
}

float wrapAngleFloatDegree(float value){
    if(value >= 2*360){
        return wrapAngleFloatDegree(value - 360);
    } else if(value < 0){
        return wrapAngleFloatDegree(value + 360);
    } else {
        return value;
    }
}

float wrapAngleFloatRadian(float value){
    if(value >= 2*PI){
        return wrapAngleFloatRadian(value - 2 * PI);
    } else if(value < 0){
        return wrapAngleFloatRadian(value + 2 * PI);
    } else {
        return value;
    }
}

void debugHeader(){
    #ifdef RECEIVER_RAW
    Serial.print(F("Ch1:")); Serial.print("\t");
    Serial.print(F("Ch2:")); Serial.print("\t");
    Serial.print(F("Ch3:")); Serial.print("\t");
    Serial.print(F("Ch4:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_PULSE
    Serial.print(F("Right_Pulse:")); Serial.print("\t");
    Serial.print(F("Left_Pulse:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_DEGREE
    Serial.print(F("Right_Deg:")); Serial.print("\t");
    Serial.print(F("Left_Deg:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_RADIAN
    Serial.print(F("Right_Rad:")); Serial.print("\t");
    Serial.print(F("Left_Rad:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_REVOLUTION
    Serial.print(F("Right_Rev:")); Serial.print("\t");
    Serial.print(F("Left_Rev:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_PPS
    Serial.print(F("Right_PPS:")); Serial.print("\t");
    Serial.print(F("Left_PPS:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_DPS
    Serial.print(F("Right_DPS:")); Serial.print("\t");
    Serial.print(F("Left_DPS:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_RPS
    Serial.print(F("Right_RPS:")); Serial.print("\t");
    Serial.print(F("Left_RPS:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_RPM
    Serial.print(F("Right_RPM:")); Serial.print("\t");
    Serial.print(F("Left_RPM:")); Serial.print("\t");
    #endif

    #ifdef TARGET_RPM
    Serial.print(F("Right_Target")); Serial.print("\t");
    Serial.print(F("Left_Target")); Serial.print("\t");
    #endif

    #ifdef PWM_RESPONSE
    Serial.print(F("Right_PWM")); Serial.print("\t");
    Serial.print(F("Left_PWM")); Serial.print("\t");
    #endif

    #ifdef ERROR_PID
    Serial.print(F("Error_Right")); Serial.print("\t");
    Serial.print(F("Error_Left")); Serial.print("\t");
    #endif

    #ifdef SUM_ERROR_PID
    Serial.print(F("Sum_Error_Right")); Serial.print("\t");
    Serial.print(F("Sum_Error_Left")); Serial.print("\t");
    #endif

    #ifdef VEHICLE_POSITION
    Serial.print(F("Vehicle_X_Pose")); Serial.print("\t");
    Serial.print(F("Vehicle_Y_Pose")); Serial.print("\t");
    Serial.print(F("Vehicle_Theta")); Serial.print("\t");
    #endif

    #ifdef VEHICLE_SPEED
    Serial.print(F("Vehicle_Speed_Right")); Serial.print("\t");
    Serial.print(F("Vehicle_Speed_Left")); Serial.print("\t");
    #endif

    #ifdef MOTOR_PULSE_DIFFERENCE
    Serial.print(F("Right_Pulse_diffference")); Serial.print("\t");
    Serial.print(F("Left_Pulse_diffference")); Serial.print("\t");
    #endif

    #ifdef EKF_DATA
    Serial.print(F("Right_Wheel_Speed")); Serial.print(",");
    Serial.print(F("Left_Wheel_Speed")); Serial.print(",");
    Serial.print(F("EKF_State")); Serial.print(",");
    #endif

    Serial.println();
}

void debug(){
    #ifdef RECEIVER_RAW
    Serial.print(ch_1_value); Serial.print("\t");
    Serial.print(ch_2_value); Serial.print("\t");
    Serial.print(ch_3_value); Serial.print("\t");
    Serial.print(ch_4_value); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_PULSE
    Serial.print(RightEncoder.getPulse()); Serial.print("\t");
    Serial.print(LeftEncoder.getPulse()); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_DEGREE
    Serial.print(RightEncoder.getAngleDeg()); Serial.print("\t");
    Serial.print(LeftEncoder.getAngleDeg()); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_RADIAN
    Serial.print(RightEncoder.getAngleRad()); Serial.print("\t");
    Serial.print(LeftEncoder.getAngleRad()); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_REVOLUTION
    Serial.print(RightEncoder.getAngleRev()); Serial.print("\t");
    Serial.print(LeftEncoder.getAngleRev()); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_PPS
    Serial.print(RightEncoder.getOmegaPPS()); Serial.print("\t");
    Serial.print(LeftEncoder.getOmegaPPS()); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_DPS
    Serial.print(RightEncoder.getOmegaDPS()); Serial.print("\t");
    Serial.print(LeftEncoder.getOmegaDPS()); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_RPS
    Serial.print(RightEncoder.getOmegaRPS()); Serial.print("\t");
    Serial.print(LeftEncoder.getOmegaRPS()); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_RPM
    Serial.print(right_rpm_filtered); Serial.print("\t");
    Serial.print(left_rpm_filtered); Serial.print("\t");
    #endif

    #ifdef TARGET_RPM
    Serial.print(right_rpm_target); Serial.print("\t");
    Serial.print(left_rpm_target); Serial.print("\t");
    #endif

    #ifdef PWM_RESPONSE
    Serial.print(right_pwm); Serial.print("\t");
    Serial.print(left_pwm); Serial.print("\t");
    #endif

    #ifdef ERROR_PID
    Serial.print(RightMotorPID.getError()); Serial.print("\t");
    Serial.print(LeftMotorPID.getError()); Serial.print("\t");
    #endif

    #ifdef SUM_ERROR_PID
    Serial.print(RightMotorPID.getSumError()); Serial.print("\t");
    Serial.print(LeftMotorPID.getSumError()); Serial.print("\t");
    #endif

    #ifdef VEHICLE_POSITION
    Serial.print(pose_x); Serial.print("\t");
    Serial.print(pose_y); Serial.print("\t");
    Serial.print(pose_theta/PI*180.0); Serial.print("\t");
    #endif
    
    #ifdef VEHICLE_SPEED
    Serial.print(velocity_right); Serial.print("\t");
    Serial.print(velocity_left); Serial.print("\t");
    #endif

    #ifdef MOTOR_PULSE_DIFFERENCE
    Serial.print(RightEncoder.getDeltaPulse()); Serial.print("\t");
    Serial.print(LeftEncoder.getDeltaPulse()); Serial.print("\t");
    #endif

    #ifdef EKF_DATA
    Serial.print(velocity_right); Serial.print(",");
    Serial.print(velocity_left); Serial.print(",");
    Serial.print(data_id);
    #endif

    Serial.println();
}
