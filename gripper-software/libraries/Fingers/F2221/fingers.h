
#pragma once

#include "RowSensor.h"

namespace hw { namespace Fingers {

//extern finger_t F1, F2;
struct finger_t {
    tactile_sensor::RowSensor<7> TactileSensor;
};
typedef std::array<finger_t*, 2> fingers_t; 
extern fingers_t F;

void init();

inline void measure_tactile() {
  for(auto f : hw::Fingers::F){
    f->TactileSensor.measure();
  }
}

inline float total_force(){
    float total_force = 0;
    for(auto f : hw::Fingers::F){
        total_force += f->TactileSensor.total_force();
    }
    return total_force;
}

}} // hw::Fingers