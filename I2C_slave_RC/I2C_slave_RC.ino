#include <Wire.h>



#define CH1_PIN 5
#define CH2_PIN 6
#define CH3_PIN 7



int ch_1;
int ch_2;
int ch_3;



int value1;
int value2;
int value3;



// 0 -> ch1 + ch2 +
// 1 -> ch1 - ch2 +
// 2 -> ch1 + ch2 -
// 3 -> ch1 - ch2 -
byte pos_neg = 100;
byte data1;
byte data2;
byte data3;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(2);
  Wire.onRequest(requestEvent);
}



void loop() {
  //pos_neg = 0;
  // put your main code here, to run repeatedly:
  ch_1 = pulseIn(CH1_PIN, HIGH, 100000);
  ch_2 = pulseIn(CH2_PIN, HIGH, 100000);
  ch_3 = pulseIn(CH3_PIN, HIGH, 100000);



 
  ch_1 = constrain(ch_1, 1000, 2000);
  ch_2 = constrain(ch_2, 1000, 2000);
  ch_3 = constrain(ch_3, 1000, 2000);
  
  //ch_1 = map(ch_1, 1000, 2000, -255, 255);
  //ch_2 = map(ch_2, 1000, 2000, -255, 255);
  ch_3 = map(ch_3, 1000, 2000, 0, 255);
  Serial.print(ch_1); Serial.print("\t");
  Serial.print(ch_2); Serial.print("\t");
  Serial.print(ch_3); Serial.print("\t");
  //Serial.println();



 value1 = to_pwm(ch_1);
  value2 = to_pwm(ch_2);
  value3 = ch_3;
  Serial.print(value1); Serial.print("\t");
  Serial.print(value2); Serial.print("\t");
  Serial.print(value3); Serial.print("\t");
  Serial.println();
  data1 = abs(value1);
  data2 = abs(value2);
  data3 = abs(value3);
  
  if(value1 >= 0 && value2 >= 0){
    pos_neg = 0;
  } else if(value1 < 0 && value2 >= 0){
    pos_neg = 1;
  } else if(value1 >= 0 && value2 < 0){
    pos_neg = 2;
  } else if(value1 < 0 && value2 < 0){
    pos_neg = 3;
  }



 //Wire.beginTransmission(2);
  //Wire.write(pos_neg);
  //Wire.write(data1);
  //Wire.write(data2);
  //Wire.write(data3);
  //Wire.endTransmission();
}



void requestEvent(){
  Wire.write(pos_neg);
  Wire.write(data1);
  Wire.write(data2);
  Wire.write(data3);
}



int to_pwm(int value){
  if(value < 1425){
    return map(value, 1425, 1000, 0, -255);  
  } else if(value > 1575){
    return map(value, 1575, 2000, 0, 255);
  } else {
    return 0;
  }
}
