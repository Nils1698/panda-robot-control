# pragma once

#include <Arduino.h>
#include "Bitcraze_PMW3901_mod.h"

class OpticalSensor : public Bitcraze_PMW3901 {

    public:
    OpticalSensor(int cs, SPIClass &spi) : Bitcraze_PMW3901(cs, spi) {}
    bool connect();
    void update();
    void resetSlide();
    float deltaX();
    float deltaY();
    float slideX(bool abs=false);
    float slideY(bool abs=false);
    float slide(bool abs=false);
    void set_conversion(float cx, float cy = -1);
    bool is_connected();

    private:
    bool _is_connected=false;
    int curX, curY, sumX, sumY, abssumX, abssumY;
    float conv_factor_x=1, conv_factor_y=1;
};