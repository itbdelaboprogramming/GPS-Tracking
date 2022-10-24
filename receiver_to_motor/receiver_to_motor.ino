/** 
  Code to read pwm from receiver X8R Pro to control motor
  Author  : Mochamad Luthfi Hariyadin
  Project : GPS Tracking
  For more information contact
  -> email: luthfihariyadin06@gmail.com
  Reference :
  -> https://www.instructables.com/4WD-Security-Robot/
**/



#define PIN_CH_1 12
#define PIN_CH_2 13

#define PIN_EN 8

#define PIN_RPWM_RIGHT 5
#define PIN_LPWM_RIGHT 6
#define PIN_RPWM_LEFT 9
#define PIN_LPWM_LEFT 10

#define MAX_PWM_MOVE 63 // 2^6-1
#define MAX_PWM_TURN 31 // 2^5-1
#define MOTOR_THRESHOLD 5

int ch1;
int ch2;
int moveValue;
int turnValue;
int pwmRight;
int pwmLeft;

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_CH_1, INPUT);
  pinMode(PIN_CH_2, INPUT);

  pinMode(PIN_EN, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(PIN_EN, HIGH);
  
  ch1 = pulseIn(PIN_CH_1, HIGH, 1000000);
  ch2 = pulseIn(PIN_CH_2, HIGH, 1000000);

  ch1 = constrain(ch1, 1000, 2000);
  ch2 = constrain(ch2, 1000, 2000);

  moveValue = map(ch1, 1000, 2000, -MAX_PWM_MOVE, MAX_PWM_MOVE);
  turnValue = map(ch2, 1000, 2000, -MAX_PWM_TURN, MAX_PWM_TURN);

  pwmRight = moveValue + turnValue;
  pwmLeft = moveValue - turnValue;

  motor_right(pwmRight);
  motor_left(pwmLeft);

  Serial.print(pwmRight);Serial.print("\t");
  Serial.print(pwmLeft);Serial.print("\t");
  //Serial.print(ch1);Serial.print("\t");
  //Serial.print(ch2);Serial.print("\t");
  Serial.println();
}

void motor_right(int pwm){
  if(pwm >= MOTOR_THRESHOLD){
    analogWrite(PIN_RPWM_RIGHT, pwm);
    analogWrite(PIN_LPWM_RIGHT, 0);
  } else if(pwm <= -MOTOR_THRESHOLD){
    analogWrite(PIN_RPWM_RIGHT, 0);
    analogWrite(PIN_LPWM_RIGHT, -pwm);
  } else {
    analogWrite(PIN_RPWM_RIGHT, 0);
    analogWrite(PIN_LPWM_RIGHT, 0);
  }
}

void motor_left(int pwm){
  if(pwm >= MOTOR_THRESHOLD){
    analogWrite(PIN_RPWM_LEFT, 0);
    analogWrite(PIN_LPWM_LEFT, pwm);
  } else if(pwm <= -MOTOR_THRESHOLD){
    analogWrite(PIN_RPWM_LEFT, -pwm);
    analogWrite(PIN_LPWM_LEFT, 0);
  } else {
    analogWrite(PIN_RPWM_LEFT, 0);
    analogWrite(PIN_LPWM_LEFT, 0);
  }
}

