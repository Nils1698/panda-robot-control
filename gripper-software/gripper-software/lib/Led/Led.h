#pragma once

#include <Arduino.h>

namespace hw {

class Led
{
private:
    const int PIN;
    bool _is_on;

    void set_state(bool new_state);

public:
    Led(int pin) : PIN(pin){};
    void init();
    void on();
    void off();
    void blink(unsigned long delay_ms = 300);

private:

};

} //hw