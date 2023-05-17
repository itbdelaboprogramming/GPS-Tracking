# Servo Control with Serial (Discontinued)

This code is used to test if direct serial communication from computer is suitable to control the movement of the rover. For this test we try to control the movement of the servo. **From the test we concluded that communication using direct serial communication isn't suitable for the project.** When using direct serial, the human detection algorithm flow is disturbed. ecause of this, we will use ROS for communicating between computer and Arduino.  

# How to Use

We will need these file to show the direct serial communication:

1. [Arduino Sketch](/MSD700_Follow_Me/Arduino/control_servo_with_serial/control_servo_with_serial.ino)
2. [Python Script](/MSD700_Follow_Me/serial_control/follow_me.py)

## Setup on Arduino Board

Just upload [this](/MSD700_Follow_Me/Arduino/control_servo_with_serial/control_servo_with_serial.ino) sketch into the Arduino Board. In this case we will use Arduino Uno R3 Board.  

Connect the servo to the Arduino Board.  

![Arduino Board](/MSD700_Follow_Me/img/servo_wiring.png?raw=true "Arduino Wiring")

## Setup on Your Machine

For this exposition, there is no need to specific setup for the machine. We just follow the steps from *[human_detection](/MSD700_Follow_Me/human_detection/README.md#how-to-use)*.

# Code Explanation

## Arduino Code

Open [this](/MSD700_Follow_Me/Arduino/control_servo_with_serial/control_servo_with_serial.ino) sketch. This code uses an Arduino board to control a servo motor based on commands received over a serial connection.

```cpp
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
```

First, we import `Servo` library and create a Servo object.

```cpp
#include <Servo.h>

Servo servo;
```

In the `setup()` function, `Serial.begin(9600)` initializes the serial communication with a baud rate of $$9600$$. `servo.attach(9)` attaches the servo motor to pin 9 of the Arduino board.

```cpp
void setup(){
    Serial.begin(9600)
    servo.attach(9);
}
```

In the `loop()` function, the code waits for serial input by checking if `Serial.available()` returns a value greater than 0. If there is serial input available, `Serial.parseInt()` reads the input and stores it in the command variable as an integer.

The if-else if statements check the value of the command variable and write the corresponding angle to the servo motor using `servo.write()`. If `command` is equal to 1, the servo motor is set to 0 degrees. If `command` is equal to 2, the servo motor is set to 180 degrees. If `command` is equal to 3, the servo motor is set to 90 degrees.

```cpp
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
```

The code will continue to read and execute commands as long as serial input is available.