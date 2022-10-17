
#pragma once 

#include <Arduino.h>
#include <TactileSensorInterface.h>

namespace tactile_sensor {

class SingleSensor : public TactileSensorInterface{
  public:
    SingleSensor(SamplingCircuit &circuit, int channel);

    void measure() override;
    const size_t n_taxels() const override {return 1;}

    float total_force() const override;
    float avg_force() const override;
    float peak_force() const override;
    location_t  peak_location() const override;
    int peaks() const override;

    void print();

    void set_calibration(float offset, float factor);

    friend Print& operator<<(Print& out, const SingleSensor &obj);

  private:
    float _current_force;
    float _c0=0,_c1=1; // Calibration constants
    const uint8_t _chnl;
};

inline Print& operator<<(Print& out, const SingleSensor& obj) {
    out.print(obj._current_force);
    return out;
}


} // tactile_sensor