# pragma once

#include <Arduino.h>
#include "USBHost_t36.h"

class OpticalSensor {

    public:
    bool connect(bool keep_on=false);
    void update();
    void resetSlide();
    float slideX(bool abs=false);
    float slideY(bool abs=false);
    float slide(bool abs=false);
    void power_on();
    void power_off();
    bool is_on();
    void set_conversion(float cx, float cy = -1);

    private:
    int sumX, sumY, abssumX, abssumY;
    bool powered_on = false;
    float conv_factor_x=1, conv_factor_y=1;

};