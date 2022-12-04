#include <Servo.h>
#include <NewPing.h>
#include "LPF.h"


#define SONAR_NUM 1      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define SERVO_PIN 13
#define RIGHT_IR 24
#define LEFT_IR 22

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(26, 28, MAX_DISTANCE), //Right Sensor
};

Servo myservo;

float distance;
float val;

// Create LPF object
int fc = 1; //fc = cut-off frequency (in Hz)
LPF dlpf(fc);

// Timestamp variables
unsigned long curr_millis;
unsigned long prev_millis;
float Ts;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.
  myservo.attach(SERVO_PIN);  // attaches the servo on pin 13 to the servo object

  Serial.print("Right_Sensor");Serial.print("\t");
  Serial.print("Left_Sensor");Serial.print("\t");
  Serial.print("Angle");Serial.print("\t");
  Serial.print("Servo");Serial.print("\t");
  Serial.println();
}

void loop() {
  // put your main code here, to run repeatedly:
  curr_millis = millis();   // Bookmark the time 

  // Start timed loop for everything else (in ms)
  if (curr_millis - prev_millis >= 10) {
    Ts = curr_millis - prev_millis;

    int right_side = digitalRead(RIGHT_IR);
    int left_side = digitalRead(LEFT_IR);

    distance = sonar[0].ping_cm();
    distance = dlpf.filter(distance, Ts/1000.0);

    if (distance >= 10 && distance <= 80){
      if (right_side == 1 && left_side == 0){
        val = 135;
      } else if (right_side == 0 && left_side == 1){
        val = 45;
      } else {
        val = 90;
      } 
    } else {
      val = 0;
    }

    myservo.write(val);                  // sets the servo position according to the scaled value
    delay(15);                           // waits for the servo to get there

    prev_millis = curr_millis;

    Serial.print(right_side);Serial.print("\t");
    Serial.print(left_side);Serial.print("\t");
    Serial.print(distance);Serial.print("\t");
    Serial.print(val);Serial.print("\t");
    Serial.println();
  }
}
