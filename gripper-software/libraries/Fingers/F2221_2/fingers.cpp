
#include "fingers.h"
#include <hw_analog_definitions.h>
#include "CircuitBoard.h"
#include "UserStream.h"

namespace hw { namespace Fingers {

finger_t F1 = {tactile_sensor::RowSensor<7>(circI, {0,1,2,3,4,5,6})};
finger_t F2 = {tactile_sensor::RowSensor<7>(circII, {0,1,2,3,4,5,6})};
fingers_t F = {{&F1,&F2}};

void init(){
    debug_stream << "F2221 (2) init\n";
    hw::analog::init();

    circI.init();
    circI.set_DAC_delay(200);
    circI.set_useAutoRange(true);

    circII.init();
    circII.set_DAC_delay(200);
    circII.set_useAutoRange(true);
}

}} // hw::Fingers