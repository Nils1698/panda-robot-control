
#pragma once

#include <Circuit_2x7.h>
#include "RowSensor.h"
#include "DistanceSensor.h"

namespace hw {

constexpr float STROKE = 94.0;

extern tactile_sensor::RowSensor<4> tact_F1;
extern tactile_sensor::RowSensor<4> tact_F2;
extern distance_sensor::DistanceSensor dist_F1;
extern distance_sensor::DistanceSensor dist_F2;

void init_fingers();
void filter_tactile(bool en);

} // hw