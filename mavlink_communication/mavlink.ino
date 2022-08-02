/** 
  Code to send data using Mavlink Protocol from Arduino
  Author  : Mochamad Luthfi Hariyadin
  Project : GPS Tracking
  Purpose : send the self-written data (latitude, longitude, and heading) from arduino to Mission Planner by using telemetry
  For more information contact
  -> email: luthfihariyadin06@gmail.com
  Reference :
  -> https://www.locarbftw.com/understanding-the-arduino-mavlink-library/
  -> https://discuss.ardupilot.org/t/mavlink-step-by-step/9629
  -> https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566
**/

// Adding mavlink library
#include "mavlink/common/mavlink.h"
/** 
  Mavlink library can be downloaded from :
  Source 1  : https://discuss.ardupilot.org/uploads/default/original/2X/0/064748b43308485aa9bd0d86fb91d74e17ed8c2a.zip
  Source 2  : https://github.com/mavlink/mavlink
**/

// ID System dan Component
#define MAVLINK_SYSTEM_ID 1 
#define MAVLINK_COMPONENT_ID 1
/** 
  System ID for Vehicle is 1, for Ground Control is 255
  Component ID for Flight Control and Ground Control is 1
  Source: https://ardupilot.org/dev/docs/mavlink-basics.html
**/

/** Timer **/ 
uint32_t tnow;
uint32_t tlast_heartbeat;
//uint32_t tlast_attitude;
//uint32_t tlast_gps;

/** Function to set execute at specific frequency **/
#define EXECUTE(TLAST, FREQ) if((tnow-TLAST) >= 1000/FREQ)
/** 
  @brief EXECUTE(TLAST, FREQ) will be execute at desired frequency
  @param TLAST the last time this function is executed in milisecond
  @param FREQ the frequency which this function will be executed in Hz
**/

void setup() {
  /** Start serial connection at 57600 baudrate **/ 
  Serial.begin(57600);

  /** Reset all timer to 0 **/
  //tlast_attitude = 0;
  //tlast_gps = 0;
  tlast_heartbeat = 0;
}

void loop() {
  /** Start timer using milisecond as unit **/
  tnow = millis();

  /** Sending heartbeat, attitude and gps data at 1 Hz **/
  EXECUTE(tlast_heartbeat, 1){
    send_heartbeat();
    send_attitude(tnow, radians(45.00), radians(0.00), radians(30.00), radians(0.00), radians(0.00), radians(0.00));
    send_gps(tnow, -68916370, 1076106390, 1000, 1000, 4500);
    tlast_heartbeat = tnow;
  }
  
  /** 
    Heartbeat is needed to informed the other that this device is exist and sending mavlink data
    send_attitude() and send_gps() are function that convert the data to format that can be sent using Mavlink {see below}
  **/
}

void send_heartbeat() {
    /** Message variable and buffer declaration for sending the heartbeat **/
    mavlink_message_t msg_send;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    /** 
      This function will fill the mavlink_message_t variable with the needed field 
      The message field for Heartbeat message can be seen at https://mavlink.io/en/messages/common.html#HEARTBEAT
    **/
    mavlink_msg_heartbeat_pack(
      MAVLINK_SYSTEM_ID,          /** System ID **/
      MAVLINK_COMPONENT_ID,       /** Component ID **/
      &msg_send,                  /** Message variable that will be filled **/
      /** Heartbeat Field **/
      10,                         /** Type of vehicle (use 10 for rover) {see -> https://mavlink.io/en/messages/common.html#MAV_TYPE}**/
      0,                          /** Type of Autopilot used (use 0 for generic autopilot) {see -> https://mavlink.io/en/messages/common.html#MAV_AUTOPILOT} **/
      0,                          /** Base system mode bitmap {see -> https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG} **/
      0,                          /** Custom mode (don't know yet what this value means, fill with 0) **/
      0                           /** System status of the vehicle (use 0 uninitialized system) {see -> https://mavlink.io/en/messages/common.html#MAV_STATE} **/
    );

    /** Continue to serialize the message sent to a buffer and send it through serial comunication **/
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_send);
    Serial.write(buf,len);
};

void send_gps(
      uint32_t t,
      int32_t lat,                /** [degE7] Latitude **/
      int32_t lon,                /** [degE7] Longitude **/
      int32_t alt,                /** [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL. **/
      int32_t relative_alt,       /** [mm] Altitude above ground **/
      uint16_t hdg                /** [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX **/
  ){
    /** Message variable and buffer declaration for sending the GPS data **/
    mavlink_message_t msg_send;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    /** 
      This function will fill the mavlink_msg_global_position_int_pack variable with the needed field 
    **/
    mavlink_msg_global_position_int_pack(
        MAVLINK_SYSTEM_ID,        /** System ID **/
        MAVLINK_COMPONENT_ID,     /** Component ID **/
        &msg_send,                /** Message variable that will be filled **/
       /** GPS Field **/
        t,                        /** [ms] Timestamp (time since system boot). **/
        lat,                      /** [degE7] Latitude, expressed **/
        lon,                      /** [degE7] Longitude, expressed **/
        alt,                      /** [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL. **/
        relative_alt,             /** [mm] Altitude above ground **/
        0,                        /** [cm/s] Ground X Speed (Latitude, positive north) **/
        0,                        /** [cm/s] Ground Y Speed (Longitude, positive east) **/
        0,                        /** [cm/s] Ground Z Speed (Altitude, positive down) **/
        hdg                       /** [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX **/
     );
    
    /** Continue to serialize the message sent to a buffer and send it through serial comunication **/
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_send);
    Serial.write(buf,len);
};

void send_attitude(
    uint32_t t,
    float roll,
    float pitch,
    float yaw,
    float rollspeed,
    float pitchspeed,
    float yawspeed
  ){
    /** Message variable and buffer declaration for sending the GPS data **/
    mavlink_message_t msg_send;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    /** 
      This function will fill the mavlink_msg_attitude_pack variable with the needed field 
    **/
     mavlink_msg_attitude_pack(
       MAVLINK_SYSTEM_ID,         /** System ID **/
       MAVLINK_COMPONENT_ID,      /** Component ID **/
       &msg_send,                 /** Message variable that will be filled **/
      /** Attitude Field **/
       t,                         /** [ms] Timestamp (time since system boot). **/
       roll,                      /** [radian] Roll **/
       pitch,                     /** [radian] Pitch **/
       yaw,                       /** [radian] Yaw **/
       rollspeed,                 /** [radian/s] Rollspeed **/
       pitchspeed,                /** [radian/s] Pitchspeed **/
       yawspeed                   /** [radian/s] Yawspeed **/
     );
    
    /** Continue to serialize the message sent to a buffer and send it through serial comunication **/
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_send);
    Serial.write(buf,len);
};
