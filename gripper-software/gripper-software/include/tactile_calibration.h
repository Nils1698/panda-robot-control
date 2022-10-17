#pragma once

#include "fingers.h"
#include "EEPROM.h"

namespace TactileCalibration {


inline void cal_save(){
    int addr = 0;

    for(auto &f : hw::Fingers::F){
      auto &sensor = f->TactileSensor;
      for (size_t i = 0; i < sensor.n_taxels(); i++)
      {
        auto params = sensor.get_calib_params(i);
        EEPROM.put(addr, params);
        addr += sizeof(params);
      }
    }

}

inline void cal_load(){
  int addr = 0;
  for(auto &f : hw::Fingers::F){
    auto &sensor = f->TactileSensor;
    for (size_t i = 0; i < sensor.n_taxels(); i++)
    {
      tactile_sensor::calib_type params;
      EEPROM.get(addr, params);
      sensor.set_calib_params(i, params);
      addr += sizeof(params);
    }
  }
}

inline void cal_save_thres(){
    int addr = 100;
    for(auto &f : hw::Fingers::F){
      auto &sensor = f->TactileSensor;
      for (size_t i = 0; i < sensor.n_taxels(); i++)
      {
        auto thres = sensor.get_threshold(i);
        EEPROM.put(addr, thres);
        addr += sizeof(thres);
      }
    }

}

inline void cal_load_thres(){
  int addr = 100;
  for(auto &f : hw::Fingers::F){
    auto &sensor = f->TactileSensor;
    for (size_t i = 0; i < sensor.n_taxels(); i++)
    {
      float thres;
      EEPROM.get(addr, thres);
      sensor.set_threshold(i,thres);
      addr += sizeof(thres);
    }
  }
}

inline void cal_reset(){
  for(auto &f : hw::Fingers::F){
    auto &sensor = f->TactileSensor;
    for (size_t i = 0; i < sensor.n_taxels(); i++){
      sensor.reset_calib_params(i);
    }
  }  
}

inline void zcal(){
  for(auto f : hw::Fingers::F){
    f->TactileSensor.measure();
    f->TactileSensor.calib_zero();
  }
}

inline void zcal_reset(){
  for(auto f : hw::Fingers::F){
    f->TactileSensor.reset_zero_calib();
  }
}

} // TactileCalibration