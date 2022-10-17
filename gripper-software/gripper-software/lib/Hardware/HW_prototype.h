#pragma once

#include "CircuitBoard.h"
#include "fingers.h"
#include "Motor.h"
#include "Button.h"
#include "Led.h"

namespace hw {

//Threads::Mutex finger_mutex, optical_mutex, motor_mutex;
extern Threads::Mutex tactile_mutex;

void init();

motor_interface::Motor *motor();
motor_interface::Motor &motor_ref();
Button *button();
Led *led();

void measure_tactile(unsigned int Ts=20); // TODO use -1 as default, change everywhere else
float current_force();

} // hw