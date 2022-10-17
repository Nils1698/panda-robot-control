#pragma once

#include "executer.h"

#include "ForceController.h"
#include "PIDaI_controller.h"
#include "detection_controller.h"

#include <vector>

namespace GripperActions {

extern exec::Executer executer;
extern ForceController * controller;
extern PID_Controller pid_controller;
extern Detect_Controller detect_controller;

void tactile_release(const float force=0, float speed=10, float max_change=-1);

enum detetion_result {
    DETECT_RES_F1=0,
    DETECT_RES_F2=1,
    DETECT_RES_BOTH=2,
    DETECT_RES_NONE=-1,
    DETECT_RES_TORQUE=-2,
    DETECT_RES_TIMEOUT=-3
};
detetion_result tactile_detect(float detection_force=0.05, float speed=50);

float tactile_squeeze();
void tactile_close(float force, float stop_force=0.2);

void hold_force(float force);
void init_slip_detector(int slip_thres, unsigned int no_slip_return, int lift_thres=-1, const bool decrease_grip=false, bool upside_down=false);
void stop_slip_detector();
float hold_detect_slip(int slip_thres, float init_force = 1, unsigned int no_slip_return=0);

} // GripperActions