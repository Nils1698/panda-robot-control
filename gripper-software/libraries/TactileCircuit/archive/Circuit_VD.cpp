#include "Circuit_VD.h"

namespace tactile_sensor {


Circuit_VD::Circuit_VD(uint8_t input_pin0, float Vd, float R_ref){
    _in_pin0 = input_pin0;
    _Vd = Vd;
    _Rref = R_ref;
}

void Circuit_VD::init(){
    pinMode(_in_pin0, INPUT);
}

float Circuit_VD::measure_conductance(unsigned int channel){
    float v_adc = hw::analog::read_voltage(_in_pin0+channel);
    return v_adc/((_Vd-v_adc)*_Rref);
}

} // tactile_sensor