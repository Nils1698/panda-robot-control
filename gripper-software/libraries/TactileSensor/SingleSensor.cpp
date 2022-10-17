#include "SingleSensor.h"
#include "Streaming.h"

namespace tactile_sensor {

SingleSensor::SingleSensor(SamplingCircuit  &circuit, int channel) : TactileSensorInterface(circuit), _chnl(channel) {

}

void SingleSensor::measure(){
    _current_force = max(0, _c0 + _c1 * circuit.measure_conductance(_chnl));
}

float SingleSensor::total_force() const{
    return _current_force;
}

float SingleSensor::avg_force() const{
    return _current_force;
}

float SingleSensor::peak_force() const{
    return _current_force;
}

int SingleSensor::peaks() const{
    return 1;
}

SingleSensor::location_t SingleSensor::peak_location() const{
    location_t loc={0,0};
    return loc;
}

void SingleSensor::print(){
    Serial << _current_force << "\n";
}

void SingleSensor::set_calibration(float offset, float factor){
    _c0 = offset;
    _c1 = factor;
}

};
