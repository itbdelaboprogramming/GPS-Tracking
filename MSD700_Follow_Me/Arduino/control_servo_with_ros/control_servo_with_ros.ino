/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono based on the human detected by
 * the camera
 * 
 * This code is based on this tutorial
 * http://wiki.ros.org/rosserial_arduino/Tutorials/Servo%20Controller
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle  nh;

Servo servo;

void servo_cb( const std_msgs::UInt8& cmd_msg){
  //servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
  //digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
  //Serial.println(cmd_msg.data);
  if(cmd_msg.data == 1){
    servo.write(0);
  } else if(cmd_msg.data == 2){
    servo.write(180);
  } else if(cmd_msg.data == 3){
    servo.write(90);
  }
}

ros::Subscriber<std_msgs::UInt8> sub("rover_command", servo_cb);

void setup(){
  //Serial.begin(9600);
  //Serial.println("Start ...");
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(9); //attach it to pin 9
}

void loop(){
  nh.spinOnce();
  //nh.loginfo("Test");
  delay(1);
}