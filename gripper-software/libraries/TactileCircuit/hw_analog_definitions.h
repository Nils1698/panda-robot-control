#pragma once

#include <Arduino.h>

static constexpr auto DAC0 = A21;
static constexpr auto DAC1 = A22;

typedef uint8_t pin_t;

namespace hw{ namespace analog {

    // Teensy 3.6
    constexpr float VCC = 3.3;
    constexpr int ADC_RES = 13;
    constexpr int DAC_RES = 12;

    // Arduino
    // const float VCC = 5.0;
    // const int ADC_RES = 10;
    // const int DAC_RES = 0;

    const int PWM_MAX = pow(2,DAC_RES);
    const float ADC_FACTOR= VCC / (pow(2,ADC_RES)-1);
    const float DAC_FACTOR= (pow(2,DAC_RES)-1) / VCC;

    inline void init(){
        analogReadResolution(ADC_RES);
        analogWriteResolution(DAC_RES);
    }

    inline float read_voltage(uint8_t pin){
        return ADC_FACTOR*analogRead(pin);
    }
    inline void write_voltage(uint8_t pin, float voltage){
        return analogWrite(pin, voltage*DAC_FACTOR);
    }
}}
