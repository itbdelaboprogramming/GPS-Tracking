#include <Servo.h>

Servo servo;

void setup(){
    Serial.begin(9600)
    servo.attach(9);
}

void loop(){
    while (Serial.available() > 0){
        int command = Serial.parseInt();

        if (command == 1){
            servo.write(0);
        } else if (command == 2){
            servo.write(180);
        } else if (command == 3){
            servo.write(90);
        }
    }
}