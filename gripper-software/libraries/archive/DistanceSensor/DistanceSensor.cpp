
#include "DistanceSensor.h"

namespace distance_sensor {

DistanceSensor::DistanceSensor(TwoWire *wire, String name){
  _name = name;
  sensor = Adafruit_VL6180X();
  _connected = sensor.begin(wire);
}

DistanceSensor::DistanceSensor(TwoWire *wire, pin_t scl, pin_t sda, String name){
  _name = name;
  sensor = Adafruit_VL6180X();
  wire->setSCL(scl);
  wire->setSDA(sda);
  _connected = sensor.begin(wire);
}

bool DistanceSensor::is_ok(){
  return _connected;
}

String DistanceSensor::name(){
  return _name;
}

void DistanceSensor::measure(){
  uint8_t range  = sensor.readRange();
  uint8_t status = sensor.readRangeStatus();
  if(status==VL6180X_ERROR_NONE && range!=255){
    valid = true;
    current_value = max(0, (range-c0)*c1);
    if(_lowpass_filter){
      filtered_value = _filter_coefs[0]*filtered_value + _filter_coefs[1]*current_value + _filter_coefs[2]*old_value;
      old_value=current_value;
      current_value = filtered_value;
    }
  }else{
    valid = false;
    current_value = -1;
  }
}

float DistanceSensor::current_distance(){
  return current_value;
}

void DistanceSensor::set_calibration(float offset, float factor){
  c0 = offset;
  c1 = factor;
}

String DistanceSensor::get_error_msg(uint8_t status){
  if(status >= VL6180X_ERROR_SYSERR_1 && status <= VL6180X_ERROR_SYSERR_5){
    return "System error";
  }
  switch(status){
    case VL6180X_ERROR_ECEFAIL:
      return "ECE failure";
    case VL6180X_ERROR_NOCONVERGE:
      return "No convergence";
    case VL6180X_ERROR_RANGEIGNORE:
      return "Ignoring range";
    case VL6180X_ERROR_SNR:
      return "Signal/Noise error";
    case VL6180X_ERROR_RAWUFLOW:
      return "Raw reading underflow";
    case VL6180X_ERROR_RAWOFLOW:
      return "Raw reading overflow";
    case VL6180X_ERROR_RANGEUFLOW:
      return "Reading underflow";
    case VL6180X_ERROR_RANGEOFLOW:
      return "Reading overflow";
  }
  return "None";
}

} // distance_sensor
