#include <Servo.h>
#include <NewPing.h>
#include <math.h>
#include "LPF.h"

float LEFT_SENSOR;
float RIGHT_SENSOR;
float val;

// Timestamp variables
unsigned long curr_millis;
unsigned long prev_millis;
float Ts;

// Create LPF object
int fc = 1; //fc = cut-off frequency (in Hz)
LPF RIGHT_lp(fc);
LPF LEFT_lp(fc);

#define SONAR_NUM 2      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define SERVO_PIN 13

 Servo myservo; //create servo object to control the servo:

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(24, 26, MAX_DISTANCE), //Right Sensor
  NewPing(28, 30, MAX_DISTANCE) // Left Sensor
};

void setup() {
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.
  myservo.attach(SERVO_PIN);  // attaches the servo on pin 13 to the servo object

  Serial.print("Right_Sensor");Serial.print("\t");
  Serial.print("Left_lSensor");Serial.print("\t");
  Serial.print("Angle");Serial.print("\t");
  Serial.println();
}

void loop() { 
  
  curr_millis = millis();   // Bookmark the time 

  // Start timed loop for everything else (in ms)
  if (curr_millis - prev_millis >= 10) {
    Ts = curr_millis - prev_millis;
  }
  
  RIGHT_SENSOR = sonar[0].ping_cm();
  LEFT_SENSOR = sonar[1].ping_cm();
  RIGHT_SENSOR = RIGHT_lp.filter(RIGHT_SENSOR, Ts/1000.0);
  LEFT_SENSOR = LEFT_lp.filter(LEFT_SENSOR, Ts/1000.0);

  if ( RIGHT_SENSOR >= 30 && RIGHT_SENSOR <= 150 && LEFT_SENSOR >= 30 && LEFT_SENSOR <= 150) {
      if (RIGHT_SENSOR < LEFT_SENSOR && abs(RIGHT_SENSOR-LEFT_SENSOR) > 20) {
        val = 45;
      } else if (RIGHT_SENSOR > LEFT_SENSOR && abs(RIGHT_SENSOR-LEFT_SENSOR) > 20){
        val =135;
      } else {
        val = 90;
      }
  } else {
    val = 0;
  }
  
  myservo.write(val);                  // sets the servo position according to the scaled value
  delay(15);                           // waits for the servo to get there

  prev_millis = curr_millis;
  
 Serial.print(RIGHT_SENSOR);Serial.print("\t");
 Serial.print(LEFT_SENSOR);Serial.print("\t");
 Serial.print(val);Serial.print("\t");
 Serial.println();
}
