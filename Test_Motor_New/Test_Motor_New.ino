#include "Motor.h"
#include "Encoder.h"
#include "LPF.h"
#include "pidIr.h"

// For Debugging, uncomment one of these
#define RECEIVER_RAW
//#define MOTOR_ANGLE_PULSE
//#define MOTOR_ANGLE_DEGREE
//#define MOTOR_ANGLE_RADIAN
//#define MOTOR_ANGLE_REVOLUTION
//#define MOTOR_SPEED_PPS
//#define MOTOR_SPEED_DPS
//#define MOTOR_SPEED_RPS
//#define MOTOR_SPEED_RPM

// Receiver PIN
#define PIN_CH_1 46
#define PIN_CH_2 48
#define PIN_CH_3 50
#define PIN_CH_4 52

// Motor PIN
#define RIGHT_MOTOR_REN_PIN 31
#define RIGHT_MOTOR_LEN_PIN 30
#define RIGHT_MOTOR_PWM_PIN 4

#define LEFT_MOTOR_REN_PIN 31
#define LEFT_MOTOR_LEN_PIN 30
#define LEFT_MOTOR_PWM_PIN 4

// Encoder PIN
#define RIGHT_ENC_PIN_A 12
#define RIGHT_ENC_PIN_B 11

#define LEFT_ENC_PIN_A 3
#define LEFT_ENC_PIN_B 2

// Constants
#define LOOP_TIME 10            // in milliseconds
#define PERIOD_TIME 20000       // in microseconds
#define RECEIVER_CUT_OFF 1      // in Hertz (Hz)
#define ENCODER_CUT_OFF 3       // in Hertz (Hz)
#define PWM_THRESHOLD 150       // in microseconds of receiver signal
#define MAX_RPM_MOVE 40         // in RPM for longitudinal movement
#define MAX_RPM_TURN 30         // in RPM for rotational movement

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

pidIr RightMotorPID(KP_RIGHT_MOTOR, KI_RIGHT_MOTOR, KD_RIGHT_MOTOR);
pidIr LeftMotorPID(KP_LEFT_MOTOR, KI_LEFT_MOTOR, KD_LEFT_MOTOR);

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
float right_rpm_target;
float left_rpm_target;
int right_pwm;
int left_pwm;

unsigned long time_now = 0;
unsigned long time_last = 0;
float dt;

void setup(){
    Serial.begin(9600);
    
    RightMotor.begin();
    LeftMotor.begin();
    
    setupPinReceiver();

    RightEncoder.start(callbackRA, callbackRB);
    LeftEncoder.start(callbackLA, callbackLB);

    debugHeader();
}

void loop(){
    time_now = millis();
    if(time_now - time_last >= LOOP_TIME){
        dt = time_now - time_last;
        getReceiverSignal();

        ch_1_filtered = Ch_1_lpf.filter(ch_1_value, dt);
        ch_2_filtered = Ch_2_lpf.filter(ch_2_value, dt);
        
        RightEncoder.measureOmega();
        LeftEncoder.measureOmega();

        right_rpm_filtered = RightRPM_lpf.filter(RightEncoder.getOmegaRPM(), dt);
        left_rpm_filtered = LeftRPM_lpf.filter(LeftEncoder.getOmegaRPM(), dt);

        if(ch_3_value <= 1250){
            // Mode HOLD
            vehicleStop();
        } else if(ch_3_value >= 1750){
            // Mode AUTO

        } else {
            // Mode MANUAL
            move_value = tuneReceiverSignaltoRPM(ch_1_filtered, MAX_RPM_MOVE);
            turn_value = tuneReceiverSignaltoRPM(ch_2_filtered, MAX_RPM_TURN);

            right_rpm_target = move_value + turn_value;
            left_rpm_target = move_value - turn_value;

            right_pwm = RightMotorPID.compute(right_rpm_target, right_rpm_filtered, dt);
            left_pwm = LeftMotorPID.compute(right_rpm_target, right_rpm_filtered, dt);

            vehicleGo(right_pwm, left_pwm);
        }

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
        return map(receiver_signal, 1500 - PWM_THRESHOLD, 1000, 0, max_rpm);
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

void debugHeader(){
    #ifdef RECEIVER_RAW
    Serial.print(F("Ch 1:")); Serial.print("\t");
    Serial.print(F("Ch 2:")); Serial.print("\t");
    Serial.print(F("Ch 3:")); Serial.print("\t");
    Serial.print(F("Ch 4:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_PULSE
    Serial.print(F("Right Pulse:")); Serial.print("\t");
    Serial.print(F("Left Pulse:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_DEGREE
    Serial.print(F("Right Deg:")); Serial.print("\t");
    Serial.print(F("Left Deg:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_RADIAN
    Serial.print(F("Right Rad:")); Serial.print("\t");
    Serial.print(F("Left Rad:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_REVOLUTION
    Serial.print(F("Right Rev:")); Serial.print("\t");
    Serial.print(F("Left Rev:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_PPS
    Serial.print(F("Right PPS:")); Serial.print("\t");
    Serial.print(F("Left PPS:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_DPS
    Serial.print(F("Right DPS:")); Serial.print("\t");
    Serial.print(F("Left DPS:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_RPS
    Serial.print(F("Right RPS:")); Serial.print("\t");
    Serial.print(F("Left RPS:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_RPM
    Serial.print(F("Right RPM:")); Serial.print("\t");
    Serial.print(F("Left RPM:")); Serial.print("\t");
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
    Serial.print(RightEncoder.getOmegaRPM()); Serial.print("\t");
    Serial.print(LeftEncoder.getOmegaRPM()); Serial.print("\t");
    #endif
    
    Serial.println();
}