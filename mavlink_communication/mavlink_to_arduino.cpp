#include <TinyGPS++.h>
#include <SoftwareSerial.h>

const int RXPin = 3, TXPin = 4;
const uint32_t GPSBaud = 9600;

uint32_t t;
uint32_t dt;
uint32_t t_start;
uint32_t t_end;
uint32_t t_now;
uint32_t timer;

TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);

  //Serial.println(F("Arduino - GPS module"));
  //Serial.print(F("Start Time")); Serial.println(millis());
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
  // put your main code here, to run repeatedly:
  if(gpsSerial.available() > 0){
    if(gps.encode(gpsSerial.read())){
      if(gps.location.isValid()){
        t_now = millis();
        if(start_flag){
          t_start = millis();
          Serial.print(F("GPS Starts reading data at "));
          Serial.print(t_start);
          Serial.println(F("ms"));
          Serial.println(timer);
          Serial.println(F("Reference :"));
          Serial.print(LAT); Serial.print("\t"); Serial.print(LON); Serial.println("\t");
          Serial.print(F("Count: ")); Serial.print("\t");
          Serial.print(F("time : ")); Serial.print("\t");
          Serial.print(F("Lat : ")); Serial.print("\t");
          Serial.print(F("Lon : ")); Serial.print("\t");
          Serial.print(F("Hdg : ")); Serial.print("\t");
          Serial.print(F("Sat : ")); Serial.print("\t");
          Serial.print(F("HDOP: ")); Serial.print("\t");
          Serial.print(F("Dist: ")); Serial.print("\t");
          Serial.print(F("DLat: ")); Serial.print("\t");
          Serial.print(F("DLon: ")); Serial.print("\t");
          Serial.print(F("Sat Val: ")); Serial.println("\t");
          start_flag = false;
        }

        if(t_now - t_start <= timer){
          Serial.print(counter); Serial.print("\t");
          Serial.print(t_now-t_start); Serial.print("\t");
          Serial.print(gps.location.lat(),7); Serial.print("\t");
          Serial.print(gps.location.lng(),7); Serial.print("\t");
          Serial.print(gps.course.deg(),2); Serial.print("\t");
          Serial.print(gps.satellites.value()); Serial.print("\t");
          Serial.print(gps.hdop.hdop()); Serial.print("\t");
          Serial.print(gps.distanceBetween(gps.location.lat(),gps.location.lng(),LAT,LON)); Serial.print("\t");
          dLat = gps.location.lat();
          dLon = gps.location.lng();
          Serial.print(dLat); Serial.print("\t");
          Serial.print(dLat); Serial.print("\t");
          Serial.print(gps.satellites.isValid()); Serial.print("\t");
          Serial.print(gps.hdop.isValid()); Serial.print("\t");
          Serial.print(gps.course.isValid()); Serial.println("\t");
          total_lat = total_lat + gps.location.lat();
          total_lon = total_lon + gps.location.lng();
          counter++;
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
          Serial.println(count_invalid);
          
        }
      } else {
        count_invalid++;
        Serial.println(F("Invalid data"));
        Serial.print(gps.location.lat()); Serial.print("\t");
        Serial.print(gps.location.lng()); Serial.print("\t");
        Serial.print(gps.course.deg(),2); Serial.print("\t");
        Serial.print(gps.satellites.value()); Serial.print("\t");
        Serial.print(gps.hdop.hdop()); Serial.println("\t");
      }
    }
  }
}
