
#pragma once 

#include <Arduino.h>
#include "SamplingCircuit.h"
#include <assert.h>
#include <iostream>

namespace tactile_sensor {

class TactileSensorInterface{
  public:
    struct location_t {
        float x,y;
    };

    virtual const size_t n_taxels() const {return 0;}

    TactileSensorInterface(SamplingCircuit  &sampling_circuit) : circuit(sampling_circuit) {
        assert(n_taxels() <= sampling_circuit.taxel_limit());
    }
    virtual void init(){
        circuit.init();
    };
    virtual void measure() = 0;

    //virtual float pressure() = 0;
    virtual float peak_force() const = 0;
    virtual float total_force() const = 0;
    virtual float avg_force() const = 0;
    virtual location_t peak_location() const = 0;
    virtual int peaks() const = 0;

    protected:
    SamplingCircuit &circuit;

    // TESTING //
    public:
    inline float test_meaure_time(){
        unsigned long t0,t1;
        constexpr int N = 100;
        t0 = millis();
        for (int i = 0; i < N; i++) measure();
        t1 = millis();
        return (float)(t1-t0)/N;
    }
};

} // tactile_sensor