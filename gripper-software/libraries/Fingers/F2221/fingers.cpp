
#include "fingers.h"
#include <hw_analog_definitions.h>
#include "CircuitBoard.h"
#include "UserStream.h"

namespace hw { namespace Fingers {

// tactile_sensor::RowSensor<7> sensor1(circI,  {2,0,4,5,3,6,1});  // tape sensor1
// tactile_sensor::RowSensor<7> sensor2(circII, {6,4,0,2,3,1,5}); // tape sensor2

// tactile_sensor::RowSensor<7> sensor1(circI, {0,2,4,6,3,5,1}); // 3D 1
// tactile_sensor::RowSensor<7> sensor2(circII, {6,4,2,0,1,3,5}); // 3D 2

finger_t F1 = {tactile_sensor::RowSensor<7>(circI, {0,2,4,6,3,5,1})};
finger_t F2 = {tactile_sensor::RowSensor<7>(circII, {6,4,2,0,1,3,5})};
fingers_t F = {{&F1,&F2}};

/*       [circI]                   [circII]
*
* <mux>  <taxel>  <mux>     <mux>  <taxel>  <mux>
*         |###|                     |###|
*  ch2--> |0|1| <--ch0       ch6--> |0|1| <--ch4
*  ch4--> |2|3| <--ch5       ch0--> |2|3| <--ch2
*  ch3--> |4|5| <--ch6       ch3--> |4|5| <--ch1
*   ch1--> |6|                ch5--> |6|
*
*/

void init(){
    debug_stream << "F2221 init\n";
    hw::analog::init();

    circI.init();
    circI.set_DAC_delay(200);
    circI.set_useAutoRange(true);

    circII.init();
    circII.set_DAC_delay(200);
    circII.set_useAutoRange(true);
}

}} // hw::Fingers