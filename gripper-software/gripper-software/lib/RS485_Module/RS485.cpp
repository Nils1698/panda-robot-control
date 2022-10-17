#include "RS485.h"

namespace hw {

void RS485::begin(long baud){
    pinMode(PIN_RE, OUTPUT);
    pinMode(PIN_DE, OUTPUT);
    _serial.begin(baud);
    enable_driver();
}

void RS485::enable_driver(){
    digitalWrite(PIN_RE, HIGH); // disable receiver (active-low)
    digitalWrite(PIN_DE, HIGH); // enable driver (active-high)
    mode = DRIVER_ACTIVE;
    delayMicroseconds(50);
}

void RS485::enable_receiver(){
    _serial.flush(); // wait for writing to finish
    digitalWrite(PIN_DE, LOW); // disable driver (active-high)
    digitalWrite(PIN_RE, LOW); // enable receiver (active-low)
    mode = RECEIVER_ACTIVE;
    delayMicroseconds(50);
}

size_t RS485::write(const uint8_t b) {
    if(mode != DRIVER_ACTIVE) enable_driver();
    return _serial.write(b);
}

int RS485::read(){
    if(mode != RECEIVER_ACTIVE) enable_receiver();
    return _serial.read();
}

int RS485::available(){
    if(mode != RECEIVER_ACTIVE) enable_receiver();
    return _serial.available();
}


} //hw