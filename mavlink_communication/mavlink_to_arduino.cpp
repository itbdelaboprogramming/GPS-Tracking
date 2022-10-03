/** 
  Code to send data using Mavlink Protocol from Arduino
  Author  : Mochamad Luthfi Hariyadin
  Project : GPS Tracking
  Purpose : Fetching GPS Data from Radiolink SE100 GPS Module and Compass
  For more information contact
  -> email: luthfihariyadin06@gmail.com
  Reference :
  -> https://github.com/mikalhart/TinyGPSPlus/tree/master/examples/BasicExample
**/

/**
 * @file mavlink_to_arduino.cpp
 * @author Mochamad Luthfi Hariyadin (luthfihariyadin06@gmail.com)
 * @brief Fetching GPS Data from Radiolink SE100 GPS Module and Compass
 * @version 0.1
 * @date 2022-08-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Adding TinyGPS++
#include <TinyGPS++.h>
/**
 * @brief Include library TinyGPS++. Library can be downloaded from Arduino IDE.
 * Open Arduino IDE -> Tools -> Manage Libraries -> Search "TinyGPSPlus by Mikal Hart"
 */

// Timer
uint32_t t;
uint32_t dt;
uint32_t t_start;
uint32_t t_end;
uint32_t t_now;
uint32_t timer;

double total_lat;
double total_lon;
double dLat;
double dLon;
double avg_lat;
double avg_lon;

int counter;
int count_invalid;

bool start_flag;
bool done_flag;

#define LAT -6.8896221
#define LON 107.6087897
#define bLat 0.0000387
#define bLon -0.0000322

struct gpsData {
  double Lat;
  double Lon;
  double Hdg;
  uint32_t Sat;
  float HDOP;
};

TinyGPSPlus gps;
gpsData gpsData;

void setup() {
  Serial.begin(9600);
  t = millis();
  t_now = 0;
  dt = 0;
  counter = 0;
  total_lat = 0;
  total_lon = 0;
  count_invalid = 0;
  timer = 60000;
  start_flag = true;
  done_flag = true;
}

void loop() {
  /** Check if serial comunication is available **/
  if(Serial.available() > 0){
    /** Check if GPS succesfully encoded data stream from GPS **/
    if(gps.encode(Serial.read())){
      t_now = millis();
      if(start_flag){
        t_start = millis();
        Serial.print(F("GPS Starts reading data at "));
        Serial.print(t_start);
        Serial.println(F("ms"));
        Serial.println(timer);
        Serial.println(F("Reference :"));
        Serial.print(LAT); Serial.print("\t"); Serial.print(LON); Serial.println("\t");
        Serial.print(F("t: ")); Serial.print("\t");
        Serial.print(F("P: ")); Serial.print("\t");
        Serial.print(F("S: ")); Serial.print("\t");
        Serial.print(F("H: ")); Serial.print("\t");
        Serial.print(F("C: ")); Serial.print("\t");
        //Serial.print(F("Count: ")); Serial.print("\t");
        //Serial.print(F("time : ")); Serial.print("\t");
        Serial.print(F("Lat : ")); Serial.print("\t");Serial.print("\t");
        Serial.print(F("Lon : ")); Serial.print("\t");Serial.print("\t");
        Serial.print(F("Hdg : ")); Serial.print("\t");
        Serial.print(F("Sat : ")); Serial.print("\t");
        Serial.print(F("HDOP: ")); Serial.print("\t");
        
        //Serial.print(F("Dist: ")); Serial.print("\t");
        //Serial.print(F("DLat: ")); Serial.print("\t");
        //Serial.print(F("DLon: ")); Serial.print("\t");
        //Serial.print(F("Sat Val: ")); Serial.println("\t");
        Serial.println("\t");
        start_flag = false;
      }
      
      
      if(t_now - t_start <= timer){
        gpsData.Lat = gps.location.lat()+bLat;
        gpsData.Lon = gps.location.lng()+bLon;
        gpsData.Hdg = gps.course.deg();
        gpsData.Sat = gps.satellites.value();
        gpsData.HDOP = gps.hdop.hdop();
        
        //Serial.print(counter); Serial.print("\t");
        Serial.print(t_now-t_start); Serial.print("\t");
        Serial.print(gps.location.isValid()); Serial.print("\t");
        Serial.print(gps.satellites.isValid()); Serial.print("\t");
        Serial.print(gps.hdop.isValid()); Serial.print("\t");
        Serial.print(gps.course.isValid()); Serial.print("\t");
        Serial.print(gpsData.Lat,7); Serial.print("\t");
        Serial.print(gpsData.Lon,7); Serial.print("\t");
        Serial.print(gpsData.Hdg,2); Serial.print("\t");
        Serial.print(gpsData.Sat); Serial.print("\t");
        Serial.print(gpsData.HDOP); Serial.print("\t");
        //Serial.print(gps.distanceBetween(gps.location.lat(),gps.location.lng(),LAT,LON)); Serial.print("\t");
        //dLat = gps.location.lat();
        //dLon = gps.location.lng();
        //Serial.print(dLat); Serial.print("\t");
        //Serial.print(dLat); Serial.print("\t");
        Serial.println("\t");
        if(gps.location.isValid()&&gps.satellites.isValid()&&gps.hdop.isValid()&&gps.course.isValid()){
          total_lat = total_lat + gpsData.Lat;
          total_lon = total_lon + gpsData.Lon;
          counter++;
        } else {
          count_invalid++;
        }
        
        
      } else if(done_flag){
        done_flag = false;
        avg_lat = total_lat/counter;
        avg_lon = total_lon/counter;
        Serial.println(F("Data selesai dikoleksi"));
        Serial.print(F("Banyak data = ")); Serial.println(counter);
        Serial.print(F("Total Lat = ")); Serial.println(total_lat,7);
        Serial.print(F("Total Lng = ")); Serial.println(total_lon,7);
        Serial.print(F("Average Lat = ")); Serial.println(total_lat/counter,7);
        Serial.print(F("Average Lng = ")); Serial.println(total_lon/counter,7); 
                 
      }
    }
  }
}