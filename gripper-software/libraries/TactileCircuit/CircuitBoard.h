
/*
** PCB board version 1
*/

#pragma once

#include "Circuit_2x7.h"

namespace hw {
    extern tactile_sensor::Circuit_2x7 circI;
    extern tactile_sensor::Circuit_2x7 circII;

    inline void init_circuit(){hw::analog::init();}
}

