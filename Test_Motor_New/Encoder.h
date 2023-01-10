/** 
  Header file of encoder code
  Author  : - Achmad Syahrul Irwansyah
            - Mochamad Luthfi Hariyadin
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  ->      : luthfihariyadin06@gmail.com
  Reference :
  -
**/

#ifndef ENCODER_H
#define ENCODER_H

#define GEAR_RATIO 532.0
#define REVOLUTION_DEGREE 360.0
#define REVOLUTION_RADIAN 6.28318531
#define S_TO_MS_CONVERTER 1000.0
#define M_TO_S_CONVERTER 60.0

#define ARRAY_SIZE 5
#define CUT_OFF_FREQ_ENC 3.0
#define PI 3.141592653

class Encoder {
    public:
    Encoder(int pin_a, int pin_b);
    void start(void(*userFuncA)(void),void(*userFuncB)(void));
    int getPinA(){return enc_pin_a;};
    int getPinB(){return enc_pin_b;};

    long getPulse(){return enc_pulse;};
    float getAngleDeg(){return (float)enc_pulse/GEAR_RATIO * REVOLUTION_DEGREE;};
    float getAngleRad(){return (float)enc_pulse/GEAR_RATIO * REVOLUTION_RADIAN;};
    float getAngleRev(){return (float)enc_pulse/GEAR_RATIO;};

    float getOmegaPPS(){return enc_omega_pps;};
    float getOmegaDPS(){return enc_omega_pps * REVOLUTION_DEGREE/GEAR_RATIO;};
    float getOmegaRPS(){return enc_omega_pps * REVOLUTION_RADIAN/GEAR_RATIO;};
    float getOmegaRPM(){return enc_omega_pps * M_TO_S_CONVERTER/GEAR_RATIO;};

    void measureOmega();

    void doEncoderA();
    void doEncoderB();

    private:
    int enc_pin_a;
    int enc_pin_b;
    volatile long enc_pulse;
    long enc_pulse_last;
    long enc_pulse_current;
    long last_ms;
    long current_ms;
    float enc_omega_pps;
}

#endif