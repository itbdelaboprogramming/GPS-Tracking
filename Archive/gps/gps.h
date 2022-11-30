/**
 * @file gps.h
 * @author Mochamad Luthfi Hariyadin (luthfihariyadin06@gmail.com)
 * @brief GPS header file to fetch data from Radiolink SE100 GPS Module
 * @version 0.1
 * @date 2022-08-04
 * 
 * @copyright Copyright (c) 2022
 * For more information contact luthfihariyadin06@gmail.com
 * Reference : 
 * https://github.com/mikalhart/TinyGPSPlus/tree/master/examples/BasicExample
 */

#ifndef __gps_h
#define __gps_h

#define gpsPort Serial

/**
 * Include TinyGPS++ library from Arduino IDE or from https://github.com/mikalhart/TinyGPSPlus
 * Download library using Arduino IDE
 * Open Arduino IDE -> Tools -> Manage Libraries -> Search "TinyGPSPlus by Mikal Hart"
 */
#include <TinyGPS++.h>

struct GPSData
{
    /* data */
    double lat;
    double lon;
    double hdg;
    int sat;
    float hdop;

    bool loc_valid;
    bool hdg_valid;
    bool sat_valid;
    bool hdop_valid;
};

TinyGPSPlus gps;

class GPS
{
private:
    /* data */
    bool start_flag;
    bool end_flag;
    uint32_t timer;
    uint32_t counter;
    uint32_t invalid_counter;
public:
    GPS();
    begin(int baudrate);
    read(uint32_t time_ms);
    GPSData data;



};

void GPS::GPS()
{
    start_flag = true;
    end_flag = true;
    timer = 60000;
    counter = 0;
    invalid_counter = 0;
}

void GPS::begin(int baudrate)
{
    gpsPort.begin(baudrate);
}

GPSData GPS::read(uint32_t time_ms)
{
    if(gpsPort.available()>0){
        if(gps.encode(gpsPort.read())){
            data.lat = gps.location.lat();
            data.lon = gps.location.lng();
            data.hdg = gps.course.value();
            data.sat = gps.satellites.value();
            data.hdop = gps.location.lat();
            data.loc_valid = gps.location.isValid();
            data.hdg_valid = gps.course.isValid();
            data.sat_valid = gps.satellites.isValid();
            data.hdop_valid = gps.hdop.isValid();
        }
    }
    return data;
}

#endif