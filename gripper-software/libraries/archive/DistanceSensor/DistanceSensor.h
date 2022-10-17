
#pragma once 

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include "Streaming.h"
#include "hw_analog_definitions.h"

#define DEBUG_DIST 0

namespace distance_sensor {

class DistanceSensor{
  public:
    DistanceSensor(TwoWire *wire, String name = "");
    DistanceSensor(TwoWire *wire, pin_t scl, pin_t sda, String name="");
    void measure();
    float current_distance();
    bool is_ok();
    String name();
    void set_calibration(float offset, float factor);

    static String get_error_msg(uint8_t status);

    void set_filter_coefs(float a0, float b0, float b1){
      _filter_coefs[0]=a0;
      _filter_coefs[1]=b0;
      _filter_coefs[2]=b1;
      _lowpass_filter=true;
    }
    void use_lowpass_filter(bool en){_lowpass_filter=en;}

   private:
    bool valid=true;

    bool _lowpass_filter = false;
    float _filter_coefs[3] = {0,1,0};

    String _name;
    Adafruit_VL6180X sensor;
    bool _connected;
    float c0=0,c1=1;
    float current_value;
    float old_value, filtered_value; // lowpass filter states
    
};

} // distance_sensor
