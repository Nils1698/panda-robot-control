
#include "PlusFinger_v1.h"

namespace hw {

tactile_sensor::Circuit_2x7 circI({A14,A15,A16,A17,A18,A19,A20,A13}, DAC1, 24, 11, {4,6,2,1,0,3,7,5});
tactile_sensor::Circuit_2x7 circII({A0,A1,A2,A3,A4,A5,A6,A7},        DAC0, 27, 12, {6,2,1,0,3,7,5,4});
tactile_sensor::RowSensor<4> tact_F1(circII, {1,0,3,2});
tactile_sensor::RowSensor<4> tact_F2(circI,  {0,2,1,3});

distance_sensor::DistanceSensor dist_F1(&Wire2, "F1");
distance_sensor::DistanceSensor dist_F2(&Wire,7,8, "F2");

void init_fingers(){

    tact_F1.init(); // Same as CircII.init()
    tact_F2.init();
    if (!dist_F1.is_ok()) Serial.println("F1 distance sensor not found");
    if (!dist_F2.is_ok()) Serial.println("F2 distance sensor not found");

    dist_F1.set_calibration(7.86, 0.9755); // right
    //dist_F1.set_filter_coefs(-0.222 , 0.611, 0.611);
    dist_F2.set_calibration(7.013, 0.9585); // left
    //dist_F2.set_filter_coefs(-0.222 , 0.611, 0.611);
    // TODO filter only works if sampled at fixed intervals!

    circI.set_DAC_delay(200);
    circII.set_DAC_delay(200);
    circI.set_FilterCoefs(0.8817, 0.0591, true);
    circII.set_FilterCoefs(0.8817, 0.0591, true);

    tact_F1.set_calibration(0, 11e-5,  55);
    tact_F1.set_calibration(1, 11e-5,  49);
    tact_F1.set_calibration(2, 11e-5, 110);
    tact_F1.set_calibration(3, 11e-5, 124);

    tact_F2.set_calibration(0, 11e-5, 135);
    tact_F2.set_calibration(1, 11e-5,  99);
    tact_F2.set_calibration(2, 11e-5, 340);
    tact_F2.set_calibration(3, 11e-5, 133);
}

void filter_tactile(bool en){
    circI.set_useFilter(en);
    circII.set_useFilter(en);
}

} // hw