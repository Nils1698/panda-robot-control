
#include "fingers.h"
#include <hw_analog_definitions.h>
#include "CircuitBoard.h"
#include "UserStream.h"

#include <SPI.h>
#include "OpticalSensor.h"

namespace hw { namespace Fingers {

finger_t F1 = {
    tactile_sensor::RowSensor<7>(circI, {5,6,3,4,0,1,2}),
    OpticalSensor(43, SPI2)
};
finger_t F2 = {
    tactile_sensor::RowSensor<7>(circII, {3,0,2,1,5,4,6}),
    OpticalSensor(50, SPI2)
};

fingers_t F = {{&F1,&F2}};

void init(){
    debug_stream << "F_OPT (demo) init\n";
    hw::analog::init();

    circI.init();
    circI.set_DAC_delay(200);
    circI.set_useAutoRange(true);

    circII.init();
    circII.set_DAC_delay(200);
    circII.set_useAutoRange(true);

    int i=0;
    for(auto f : hw::Fingers::F){
        if(f->opticalSensor.connect()){
            debug_stream<<"optical sensor "<<i<<" connected\n";
        }else{
            debug_stream<<"optical sensor "<<i<<" connection failed!\n";
        }
        i++;
    }


}

}} // hw:Fingers