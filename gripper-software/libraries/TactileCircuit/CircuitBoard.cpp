
#include "CircuitBoard.h"

#if CIRCUIT_VERSION==1 // Birds nest
namespace hw {
    tactile_sensor::Circuit_2x7 circI({A14,A15,A16,A17,A18,A19,A20,A13}, DAC1, 24, 11, {4,6,2,1,0,3,7,5});
    tactile_sensor::Circuit_2x7 circII({A0,A1,A2,A3,A4,A5,A6,A7},        DAC0, 27, 12, {6,2,1,0,3,7,5,4});
}
#elif CIRCUIT_VERSION==2 // PCB
namespace hw {
    tactile_sensor::Circuit_2x7 circI({A14,A15,A16,A17,A18,A19,A20,A13}, DAC0, 24, 12, {4,6,2,1,0,3,7,5});
    tactile_sensor::Circuit_2x7 circII({A0,A1,A2,A3,A4,A5,A6,A7},        DAC1, 27, 11, {6,2,1,0,3,7,5,4});
}
#endif
