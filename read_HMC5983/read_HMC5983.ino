#include <Wire.h>

#define HMC5983_ADDRESS 0x1E

int x_msb;
int x_lsb;
int z_msb;
int z_lsb;
int y_msb;
int y_lsb;

float hx;
float hz;
float hy;

float heading;

void setup(){
    Serial.begin(9600);
    Wire.begin();
    Serial.println("Start Listening ...");
}

void loop(){
    write_hmc5983(0x00, 0x10);
    write_hmc5983(0x01, 0x20);
    write_hmc5983(0x02, 0x01);
    read_hmc5983(0x03);
    delay(6);

    Wire.requestFrom(HMC5983_ADDRESS, 6);
    while(Wire.available()){
        x_msb = Wire.read();
        x_lsb = Wire.read();
        z_msb = Wire.read();
        z_lsb = Wire.read();
        y_msb = Wire.read();
        y_lsb = Wire.read();
    }
    //debug_output();
    delay(67);

    hx = (x_msb << 8) + x_lsb;
    hz = (z_msb << 8) + z_lsb;
    hy = (y_msb << 8) + y_lsb;

    if(hx > 0x07FF) hx = 0xFFFF - hx;
    if(hz > 0x07FF) hz = 0xFFFF - hz;
    if(hy > 0x07FF) hy = 0xFFFF - hy;

    heading = atan2(hy, hx) * 180 / PI;
}

void write_hmc5983(int reg, int val){
    Wire.beginTransmission(HMC5983_ADDRESS);
    Wire.write(0x3C);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void read_hmc5983(int reg){
    Wire.beginTransmission(HMC5983_ADDRESS);
    Wire.write(0x3D);
    Wire.write(reg);
    Wire.endTransmission();
}

void debug_output(){
    Serial.print(x_msb, HEX); Serial.print("\t");
    Serial.print(x_lsb, HEX); Serial.print("\t");
    Serial.print(z_msb, HEX); Serial.print("\t");
    Serial.print(z_lsb, HEX); Serial.print("\t");
    Serial.print(y_msb, HEX); Serial.print("\t");
    Serial.print(y_lsb, HEX); Serial.print("\t");
    Serial.println();
}

void debug_h(){
    Serial.print(hx, HEX); Serial.print("\t");
    Serial.print(hz, HEX); Serial.print("\t");
    Serial.print(hy, HEX); Serial.print("\t");
    Serial.println();
}

void debug_heading(){
    Serial.print(heading); Serial.print("\t");
    Serial.println();
}