/** 
  Code to test wheel-odometry reading
  
  Author  : Achmad Syahrul Irwansyah
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
**/

#include "encoder.h"

#define LOOPTIME 10 //in ms

//#define DEBUG
#define DEBUG_ODOM

// Encoder pin assignment (just use 2, 3, 10, 11, 12)
Encoder enc_kiri(2,3); // Encoder(int pin_a, int pin_b);
Encoder enc_kanan(11,12);

// Encoder callback function
void callbackKiA(){enc_kiri.doEncoderA();}
void callbackKiB(){enc_kiri.doEncoderB();}
void callbackKaA(){enc_kanan.doEncoderA();}
void callbackKaB(){enc_kanan.doEncoderB();}

unsigned long currentMillis;
unsigned long previousArmMillis;
unsigned long previousMillis;

float AngLCur = 0;
float AngRCur = 0;

float AngLPrev = 0;
float AngRPrev = 0;

float posX = 0;
float posY = 0;
float Theta = 0;

float l = 33.0; //Wheel Distance in cm
float r = 5.0; //Wheel Radius in cm

void setup() {
  Serial.begin(9600);

  enc_kiri.start(callbackKiA, callbackKiB);
  enc_kanan.start(callbackKaA, callbackKaB);

  #ifdef DEBUG
  Serial.print(F("Left_Pulse:")); Serial.print("\t");
  Serial.print(F("Right_Pulse:")); Serial.print("\t");
  Serial.print(F("Left_Angle:")); Serial.print("\t");
  Serial.print(F("Right_Angle:")); Serial.print("\t");
  Serial.println();
  #endif

  #ifdef DEBUG_ODOM
  Serial.print(F("Left_Angle:")); Serial.print("\t");
  Serial.print(F("Right_Angle:")); Serial.print("\t");
  Serial.print(F("Pose_X:")); Serial.print("\t");
  Serial.print(F("Pose_Y:")); Serial.print("\t");
  Serial.print(F("Pose_Theta:")); Serial.print("\t");
  Serial.println();
  #endif
}


void loop() {
  currentMillis = millis();   // bookmark the time 
  if (currentMillis - previousMillis >= 10) {  // start timed loop for everything else
         previousMillis = currentMillis;
         AngLCur = enc_kiri.getPos()/532.0*2.0*PI; //in rad
         AngRCur = enc_kanan.getPos()/532.0*2.0*PI;

         //Angle change rate
         double dAngL = AngLCur - AngLPrev;
         double dAngR = AngRCur - AngRPrev;
         double a = (dAngR + dAngL)/(dAngR - dAngL)*l/2.0;

         //Updating the pose of the vehicle for diff. drive
         if (dAngL == dAngR) {
            posX = posX + r/2.0*(dAngR + dAngL)*cos(Theta);
            posY = posY + r/2.0*(dAngR + dAngL)*cos(Theta);
         } else if (dAngL == -dAngR) {
            Theta = Theta + (dAngR - dAngL)*r/l;
         } else {
            posX = posX - a*sin(Theta) + a*sin(Theta + (dAngR - dAngL)*r/l);
            posY = posY + a*cos(Theta) - a*cos(Theta + (dAngR - dAngL)*r/l);
            Theta = Theta + (dAngR - dAngL)*r/l;
         }

         AngLPrev = AngLCur;
         AngRPrev = AngRCur;

         #ifdef DEBUG
         Serial.print(enc_kiri.getPos()); Serial.print("\t");
         Serial.print(enc_kanan.getPos()); Serial.print("\t");
         Serial.print(AngLCur/PI*180.0); Serial.print("\t");
         Serial.print(AngRCur/PI*180.0); Serial.print("\t");
         Serial.println();
         #endif

         #ifdef DEBUG_ODOM
         Serial.print(AngLCur/PI*180.0); Serial.print("\t");
         Serial.print(AngRCur/PI*180.0); Serial.print("\t");
         Serial.print(posX); Serial.print("\t");
         Serial.print(posY); Serial.print("\t");
         Serial.print(Theta/PI*180.0); Serial.print("\t");
         Serial.println();
         #endif
  }
  // put your main code here, to run repeatedly:

}
