#pragma once

#include "fingers.h"

#include <vector>
#include <numeric> // accumulate
#include <algorithm> // for_each

namespace TactilePerception {


int compute_contact_points(hw::Fingers::finger_t *finger);


}//TactilePerception