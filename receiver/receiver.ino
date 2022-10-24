#include "receiver.h"

#define PIN_RECEIVER 2

receiver throttle(PIN_RECEIVER);

void riser(){throttle.rising();};

void setup(){
    Serial.begin(9600);

    Serial.println("Mulai");
    throttle.start(riser);
}

void loop(){
    
}
