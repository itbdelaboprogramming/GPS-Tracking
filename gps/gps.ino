/**
 * @file gps.ino
 * @author Mochamad Luthfi Hariyadin (luthfihariyadin06@gmail.com)
 * @brief GPS header file to fetch data from Radiolink SE100 GPS Module
 * @version 0.1
 * @date 2022-08-04
 * 
 * @copyright Copyright (c) 2022
 * For more information contact luthfihariyadin06@gmail.com
 * Reference : 
 * https://github.com/mikalhart/TinyGPSPlus/tree/master/examples/BasicExample
 * 
 * This code run on Arduino Mega
 * To run this code, please mind the wiring for the gps to arduino.
 * Connect the GPS module RX-TX to Arduino's RX1-TX1 (Serial1)
 */

// To enter debug mode uncomment this line below. It will print the GPS data read from GPS.
#define DEBUG

/**
 * Include TinyGPS++ library from Arduino IDE or from https://github.com/mikalhart/TinyGPSPlus
 * Download library using Arduino IDE
 * Open Arduino IDE -> Tools -> Manage Libraries -> Search "TinyGPSPlus by Mikal Hart"
 */
#include <TinyGPS++.h>

// Declare gps object from TinyGPS++ library
TinyGPSPlus gps;

// Define gpsData struct for handling the gps data.
typedef struct gpsData {
  double latitude;
  double longitude;
  double heading;
  uint32_t satellites;
  float HDOP;
};

// Declare gpsData struct
gpsData gpsData;

void setup() {
  // Begin Serial comunication
  // Serial is for debugging and sending data to Computer
  // Serial1 is for receiving data from GPS Module
  Serial.begin(9600);
  Serial1.begin(9600);

  // This will only run in debug mode
  #ifdef DEBUG
  Serial.print(F("Latitude:")); Serial.print("\t");
  Serial.print(F("Longitude:")); Serial.print("\t");
  Serial.print(F("Heading:")); Serial.print("\t");
  Serial.print(F("Num of Sat:")); Serial.print("\t");
  Serial.print(F("HDOP:")); Serial.print("\t");
  Serial.println();
  #endif
}

void loop() {
  // Checking Serial1 port from the GPS Module
  if(Serial1.available()>1){
    // Checking if the NMEA data from Serial1 port is succesfully encoded by gps object.
    if(gps.encode(Serial1.read())){
        // Checking if location data from the GPS is in valid format
      if(gps.location.isValid()){
        // Assign all the GPS data read from the GPS module to gpsData struc
        gpsData.latitude = gps.location.lat();
        gpsData.longitude = gps.location.lng();
        gpsData.heading = gps.course.deg();
        gpsData.satellites = gps.satellites.value();
        gpsData.HDOP = gps.hdop.hdop();

        // Print all the data read from GPS to serial monitor in Debug Mode
        #ifdef DEBUG
        Serial.print(gpsData.latitude); Serial.print("\t");
        Serial.print(gpsData.longitude); Serial.print("\t");
        Serial.print(gpsData.heading); Serial.print("\t");
        Serial.print(gpsData.satellites); Serial.print("\t");
        Serial.print(gpsData.HDOP); Serial.print("\t");
        Serial.println();
        #endif
      }
    }
  }
}
