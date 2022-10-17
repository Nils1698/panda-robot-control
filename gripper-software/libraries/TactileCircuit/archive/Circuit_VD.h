
#pragma once 

#include <Arduino.h>
#include "SamplingCircuit.h"
#include "hw_analog_definitions.h"

namespace tactile_sensor {

class Circuit_VD : public SamplingCircuit{
    private:

    public:
    Circuit_VD(uint8_t input_pin, float Vd, float R_ref);
    void init() override;
    float measure_conductance(unsigned int channel) override;
    const size_t taxel_limit() override {return 0;}

    private:
    int _in_pin0;
    float _Vd, _Rref;
};

} // tactile_sensor