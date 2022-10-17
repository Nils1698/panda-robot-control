
#pragma once 

#include <Arduino.h>

namespace tactile_sensor {

class SamplingCircuit{
  public:
    virtual void init() = 0;
    virtual float measure_conductance(unsigned int taxel) = 0;
    virtual const size_t taxel_limit() = 0;
};

} // tactile_sensor