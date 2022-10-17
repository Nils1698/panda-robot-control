
#pragma once 

#include <Arduino.h>
#include "SamplingCircuit.h"
#include "hw_analog_definitions.h"
#include "Streaming.h"

#include <array>

namespace tactile_sensor {

class Circuit_2x7 : public SamplingCircuit{
    private:
    static constexpr unsigned int CHANNELS = 7;
    static constexpr float R_REF = 330; // ohm
    // static constexpr float V_DAC_INIT = 0.2; // V  (default)
    static constexpr float V_DAC_INIT = 0.02; // V  (interlink, ohmite, dfrobot)
    // static constexpr float V_DAC_INIT = 0.002; // V (tacterion)
    static constexpr float V_ADC_DES = 0.9*hw::analog::VCC; // V

    static constexpr float R_K = 1000; // Howland pump resistor

    // constexpr static const float VOLTAGE_LEVELS[] = {0.206, 0.825, 3.3}; // V
    // constexpr static const float CURRENT_LEVELS[] = {0.4125, 1.65, 6.6}; // mA
    // constexpr static const float RESISTOR_LEVELS[] = {8000, 2000, 500}; // ohm

    float v_dac_init = 0.2;
    unsigned int DAC_delay = 5000;
    int _ctrl_pin0, _dac_pin, _en_pin;
    std::array<uint8_t, CHANNELS+1> _mux_order;
    std::array<pin_t, CHANNELS+1> _input_pins;
    bool _use_autorange = true;
    bool _use_filter = false;
    float _filt_a=0, _filt_b=0;
    std::array<float, CHANNELS+1> _filt_G, _filt_U;

    public:
    Circuit_2x7(std::array<pin_t, CHANNELS+1> input_pins, pin_t dac_pin, pin_t ctrl_pin0, pin_t enable_pin, std::array<uint8_t, CHANNELS+1> mux_order);
    void init() override;
    float measure_conductance(unsigned int taxel) override;
    void check_reference();
    const unsigned int taxel_limit() override {return CHANNELS;}

    std::array<float,CHANNELS+1> read_resistances();
    void set_DAC_delay(unsigned int micros);
    void set_useAutoRange(bool en);
    void set_useFilter(bool en);
    void set_FilterCoefs(float a, float b, bool enable=false);

    void test_resitance(std::array<float,CHANNELS+1> known_res);
    void test_order(unsigned int taxel);
    void find_order();

    inline void set_VInit(float v){
        v_dac_init = v;
    }

    private:
    // float read_conductance(float v_dac, pin_t input_pin);
    void set_enabled(bool en);

    public: // DEBUG (make private again)
    void debug_non_func();

    //inline pin_t input_pin(uint8_t idx){return _in_pin0+idx;}
    inline pin_t input_pin(uint8_t idx){return _input_pins[idx];}
    inline pin_t  ctrl_pin(uint8_t idx){return _ctrl_pin0+idx;}
    inline uint8_t channel(uint8_t idx){return _mux_order[idx];}
    inline unsigned int ref_index(){return CHANNELS;}

    void set_channel(unsigned int chn);
};

} // tactile_sensor