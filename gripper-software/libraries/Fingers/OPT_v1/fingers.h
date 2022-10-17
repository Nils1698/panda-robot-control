
#pragma once

#include "RowSensor.h"
#include "OpticalSensor.h"

namespace hw { namespace Fingers {

const size_t N_TAXELS = 7;

struct finger_t {
    tactile_sensor::RowSensor<N_TAXELS> TactileSensor;
    OpticalSensor opticalSensor;
};
typedef std::array<finger_t*, 2> fingers_t; 
extern fingers_t F;

const float taxel_xy[N_TAXELS][2] =
    {{-2.75,30},{2.75,30},
     {-2.75,20},{2.75,20},
     {-2.75,10},/*ops sens*/
     {-2.75, 0},{2.75, 0}};

const float opt_xy[1][2] = {{2.75, 10}};

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


}} // hw:Fingers