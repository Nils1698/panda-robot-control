#pragma once

#include <Arduino.h>

namespace hw {

class Button
{

public:
    enum event_t {
        NONE, PRESS, RELEASE, CLICK, LONG_CLICK, HOLD, DOUBLE_CLICK
    };

private:
    struct event_details_t {
        uint32_t time, time2;
        event_t type;
    };

    const int PIN;
    const uint32_t HOLD_TIME = 500; //ms
    const uint32_t DOUBLE_CLICK_TIME = 400; //ms
    event_details_t last_button_event = {0, NONE};

    bool buttonState=false, newButtonState=false;
    uint32_t  debounceInit;
    const uint32_t DEBOUNCE_DELAY = 50;

public:
    Button(int pin) : PIN(pin){};
    void init();
    event_t read_events();

private:
    void update_state();
    bool is_pressed();

};

} //hw