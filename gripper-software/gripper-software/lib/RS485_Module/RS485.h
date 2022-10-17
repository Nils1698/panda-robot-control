#pragma once

#include <Arduino.h>

namespace hw {

class RS485 : public Stream
{
private:
    HardwareSerial &_serial;
    const int PIN_RE,  // receiver-enable (active-low)
                PIN_DE; // driver-enable (active-high)
    
    enum duplex_mode_t {
        DRIVER_ACTIVE,
        RECEIVER_ACTIVE
    };
    duplex_mode_t mode;

public:
    RS485(HardwareSerial &serial, int DE, int RE) : _serial(serial), PIN_RE(RE), PIN_DE(DE) {};
    void begin(long baud);

    size_t write(const uint8_t b) override;
    int read() override;
    int available() override;
    int peek() override {return _serial.peek();};
    operator bool()	{ return true; }

private:
    void enable_driver();
    void enable_receiver();

};

} //hw