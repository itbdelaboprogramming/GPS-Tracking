# Servo Control with Serial (Discontinued)

This code is used to test if direct serial communication from computer is suitable to control the movement of the rover. For this test we try to control the movement of the servo. **From the test we concluded that communication using direct serial communication isn't suitable for the project.** When using direct serial, the human detection algorithm flow is disturbed. ecause of this, we will use ROS for communicating between computer and Arduino.  

* [Servo Control with Serial (Discontinued)](#servo-control-with-serial-discontinued)
* [How to Use](#how-to-use)
    * [Setup on Arduino Board](#setup-on-arduino-board)
    * [Setup on Your Machine](#setup-on-your-machine)
* [Code Explanation](#code-explanation)
    * [Arduino Code](#arduino-code)
    * [Python Script](#python-script)

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

In the `setup()` function, `Serial.begin(9600)` initializes the serial communication with a baud rate of 9600. `servo.attach(9)` attaches the servo motor to pin 9 of the Arduino board.

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

## Python Script

We use *python* to send the command through seriThe code will run a detection algorithm and send the appropriate command to arduino board. Te explanation for   the detection algorithm is same as [this](/MSD700_Follow_Me/human_detection/README.md#human-detection).

```python
from device_camera import *
from darknet_yolo import *
import argparse
import serial
import time

parser = argparse.ArgumentParser(
    description= 'This program will detect human',
    epilog= 'Hope this works'
)
group = parser.add_mutually_exclusive_group()
group.add_argument('-c', '--camera', type=int, default=0, help='Camera device id')
group.add_argument('-i', '--image', action='store_true', help='Image directory')
args = parser.parse_args()
#print(args)

if args.image:
    image_path = input('Enter the path to the image: ')
    print(image_path)

net = DarknetDNN()
camera = DeviceCamera(device_id=args.camera)

#Serial Object
port = '/dev/ttyUSB0'
ser = serial.Serial(port, 9600, timeout=1)
ser.reset_input_buffer()
timestamp = time.time()
frequency = 10 # in Hz

while True:
    #Get frame from camera
    frame = camera.get_frame()

    #Detect human from the frame
    net.detect_object(frame)
    
    #Draw bounding box of the human detected
    net.draw_object(frame)

    if time.time() - timestamp >= 1/frequency:
        direct = net.get_command()

        if direct == 'Right':
            command = '1'
        elif direct == 'Left':
            command = '2'
        elif direct == 'Center':
            command = '3'
        else:
            command = '0'
        
        ser.write(command.encode("utf-8"))
        timestamp = time.time()

    #Draw grid
    camera.create_grid()

    #Display the image
    camera.show()

    #exit condition
    key = cv2.waitKey(1)
    if key == 27:
        print(f"Key {key} is pressed.")
        break

camera.release()
```

First we import the library that we needed.

```python
import serial
import time
```

The `serial` library is used to create Serial object that will handle direct serial communication in python. The `time` library will be used to set the frequency of the command that we will send.

```python
#Serial Object
port = '/dev/ttyUSB0'
ser = serial.Serial(port, 9600, timeout=1)
ser.reset_input_buffer()
timestamp = time.time()
frequency = 10 # in Hz
```

To create the serial object, we need to specify the port that we will use to communicate with arduino, for this case the port that we will use is `/dev/ttyUSB0`. The next thing we need to specify is the baud rate of the serial communication. In [this skecth](#arduino-code), we specify that we will use 9600 as baudrate.  

The `Serial` class is imported from the `serial` library, which provides support for serial communication on a computer or microcontroller. The `Serial` class constructor takes three arguments: the port name, the baud rate, and the timeout. `port` is a string that specifies the name of the serial port to which the device is connected. For example, on a Linux system, the port name might be `/dev/ttyACM0`, while on a Windows system, it might be `COM1`. The `9600` argument specifies the baud rate for the serial communication. Baud rate is the rate at which bits are transmitted over the serial communication channel. The `timeout` argument specifies the time in seconds to wait for a response from the device. If no data is received within the timeout period, the read operation will be aborted. The `reset_input_buffer()` method clears the input buffer for the serial connection. This ensures that any data received before the code begins reading from the serial connection is discarded, preventing it from interfering with the current operation.

The `time.time()` function is called to get the current time as a floating-point number of seconds since the epoch. The timestamp variable is assigned this value. The `frequency` variable is assigned a value of 10, which represents the frequency in Hertz at which the code will perform some operation. In this case, the frequency is set to 10 Hz, which means the operation will be performed 10 times per second.

```python
while True:
    ...

    if time.time() - timestamp >= 1/frequency:
        direct = net.get_command()

        if direct == 'Right':
            command = '1'
        elif direct == 'Left':
            command = '2'
        elif direct == 'Center':
            command = '3'
        else:
            command = '0'
        
        ser.write(command.encode("utf-8"))
        timestamp = time.time()
    
    ...
```

This block of code checks whether the difference between the current time `(time.time())` and the `timestamp` variable is greater than or equal to the inverse of the `frequency` variable (i.e., the time between each loop iteration). If the condition is true, the code proceeds to execute the following lines:

`direct = net.get_command()`: This calls a function `get_command()` from an object called net which returns a string representing the direction to which a camera is pointing. The code then checks the value of `direct` and assigns a corresponding value to the `command` variable. The `ser.write()` function writes the value of `command` to a serial port using the `write()` function of the `serial.Serial` object `ser`. Finally, the `timestamp` variable is updated with the current time `(time.time())` to prepare for the next iteration of the loop.