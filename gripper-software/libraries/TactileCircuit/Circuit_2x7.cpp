#include "Circuit_2x7.h"
#include "TeensyThreads.h"
#include "UserStream.h"

namespace tactile_sensor {

Circuit_2x7::Circuit_2x7(std::array<pin_t, CHANNELS+1> input_pins, pin_t dac_pin, pin_t ctrl_pin0, pin_t enable_pin, std::array<uint8_t, CHANNELS+1> mux_order)
{
  _input_pins = input_pins;
  _ctrl_pin0 = ctrl_pin0;
  _dac_pin = dac_pin;
  _en_pin = enable_pin;
  _mux_order = mux_order;
}

void Circuit_2x7::init(){
  debug_stream<<"Circ init\n";
  debug_stream<<"v_dac_init = " << _FLOAT(v_dac_init,4) << "\n";
  _filt_G.fill(0);
  _filt_U.fill(0);

  for(unsigned int i=0; i<CHANNELS; ++i){
    pinMode(input_pin(i), INPUT);
  }
  for(unsigned int i=0; i<3; ++i){
    pinMode(ctrl_pin(i), OUTPUT);
  }

  pinMode(_en_pin, OUTPUT);
  pinMode(_dac_pin, OUTPUT);
  
  set_enabled(true);
  check_reference();
}

void Circuit_2x7::set_enabled(bool en){
  digitalWrite(_en_pin, en ? LOW : HIGH);
}
void Circuit_2x7::set_useAutoRange(bool en){
  _use_autorange = en;
}

void Circuit_2x7::set_DAC_delay(unsigned int micros){
  DAC_delay = micros;
}

void Circuit_2x7::set_useFilter(bool en){
  _use_filter=en;
}

void Circuit_2x7::set_FilterCoefs(float a, float b, bool enable){
  _filt_a = a;
  _filt_b = b;
  _use_filter=enable;
}

void Circuit_2x7::set_channel(unsigned int chn){
  digitalWrite(ctrl_pin(0), (chn&0b001)>0 ? HIGH : LOW);
  digitalWrite(ctrl_pin(1), (chn&0b010)>0 ? HIGH : LOW);
  digitalWrite(ctrl_pin(2), (chn&0b100)>0 ? HIGH : LOW);
}

float Circuit_2x7::measure_conductance(unsigned int taxel){
    set_channel( channel(taxel) );

    float v_dac = v_dac_init;
    hw::analog::write_voltage(_dac_pin, v_dac);
    threads.yield();
    delayMicroseconds(DAC_delay);
    float v_adc = hw::analog::read_voltage( input_pin(taxel) );

    // Auto range
    // if(V_ADC_DES/v_adc > 1.2){
    if(_use_autorange){
      v_dac = min(v_dac_init * V_ADC_DES/v_adc, hw::analog::VCC);
      hw::analog::write_voltage(_dac_pin, v_dac);
      threads.yield();
      delayMicroseconds(DAC_delay);
    }
    // }

    v_adc = hw::analog::read_voltage( input_pin(taxel) );
    const float I = 2*v_dac/R_K;

    if(_use_filter){
      const float u = I/v_adc;
//      if(v_adc<0.99*hw::analog::VCC){
      _filt_G[taxel] = _filt_a*(u + _filt_U[taxel]) + _filt_b*_filt_G[taxel];
      _filt_U[taxel] = u;
      // }else{
      //   _filt_G[taxel] = 0;
      // }
    }else{
      _filt_U[taxel] = I/v_adc;
      _filt_G[taxel] = _filt_U[taxel];
    }
    return _filt_G[taxel];
  // return I/v_adc;
}

void Circuit_2x7::check_reference(){
  Serial << "Index: " << ref_index() << ", Channel: " << channel(ref_index()) << ", Input: " << input_pin(ref_index()) << "\n";
  float val = measure_conductance(ref_index());
  Serial << "Ref. cond.: " << val*1000000 << "uS, res: " << 1.0/val << "Ohm\n";
  // TODO
}

std::array<float,Circuit_2x7::CHANNELS+1> Circuit_2x7::read_resistances(){
  std::array<float,CHANNELS+1> result;
  for (size_t i = 0; i < CHANNELS+1; i++)
  {
    result[i] = 1.0/measure_conductance(i);
  }
  return result;
}

void Circuit_2x7::test_resitance(std::array<float,CHANNELS+1> known_res){
  float total_err = 0;
  for (size_t i = 0; i < CHANNELS+1; i++)
  {
    const float R = 1.0/measure_conductance(i);
    const float err = (R-known_res[i])/known_res[i];
    total_err += abs(err);
    Serial << "R"<<i<<": " << R << "  err: " << err*100 << "%\n";
    // Serial << err << " ";
  }
  // Serial << "\n";
  Serial << "Avg error: " << total_err/(CHANNELS+1)*100<< "%\n";

}

void Circuit_2x7::test_order(unsigned int taxel){
    set_channel( channel(taxel) );
    hw::analog::write_voltage(_dac_pin, hw::analog::VCC);
    delayMicroseconds(DAC_delay);
    Serial << "Index "<<taxel<< ", Channel "<<channel(taxel)<<"\n";
    for (size_t i = 0; i < CHANNELS+1; i++)
    {
      float v_adc = hw::analog::read_voltage( input_pin(i) );
      if(taxel==i)
        Serial << "(" << v_adc << ")\t";
      else
        Serial << v_adc << "\t";
    }
    Serial << "\n";
}

void Circuit_2x7::find_order(){
  for (size_t chn = 0; chn < CHANNELS+1; chn++){
    set_channel(chn);
    hw::analog::write_voltage(_dac_pin, hw::analog::VCC);
    delayMicroseconds(DAC_delay);
    Serial << "Channel " << chn << ": ";
    bool found = false;
    for (size_t i = 0; i < CHANNELS+1; i++)
    {
      float v_adc = hw::analog::read_voltage( input_pin(i) );
      if(v_adc > 0.99*hw::analog::VCC){
        Serial << "pin " << input_pin(i) << " ("<<v_adc<<"V)" << "\t";
        found = true;
      }
      delay(2);
    }
    if(!found) Serial << " <none>";
    Serial << "\n";
    delay(10);
  }
  
}


void Circuit_2x7::debug_non_func(){}

} // tactile_sensor