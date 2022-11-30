#include "motor.h"

Motor motor_kiri(7,8,9); // Motor(int RPWM, int LPWM, int EN);

// Timestamp variables
unsigned long curr_millis;
unsigned long prev_millis;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Start to set the pin mode
  motor_kiri.start();

  motor_kiri.debug();

}

void loop() {
  // put your main code here, to run repeatedly:
  curr_millis = millis();   // Bookmark the time 

  // Start timed loop for everything else (in ms)
  if (curr_millis - prev_millis >= 10) {
    // Set motor enable
    bool ena_ki = 1;
    motor_kiri.setEnable(ena_ki);

    // Print Angle and Omega

    // Compute pwm (uncomment what is needed)
    //int pwm_ki = pid_left_angle.compute(target_angle,curr_left_angle,max_pwm);
    //int pwm_ka = pid_left_angle.compute(target_angle,curr_right_angle,max_pwm);;

    //int pwm_ki = pid_left_omega.compute(target_omega,left_omega,max_pwm);
    //int pwm_ka = pid_left_omega.compute(target_omega,right_omega,max_pwm);

    int pwm_ki = 100;

    // Rotate motor
    motor_kiri.rotate(pwm_ki);
  }
}
