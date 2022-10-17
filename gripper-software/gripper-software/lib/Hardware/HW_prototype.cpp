
#include "HW_prototype.h"

namespace hw {

// Threads::Mutex finger_mutex, optical_mutex, motor_mutex;
Threads::Mutex tactile_mutex;

constexpr int BUTTON_PIN = /*2*/ A9, LED_PIN = 13;
Button _button(BUTTON_PIN);
Led _led(LED_PIN);
motor_interface::Motor _motor;

unsigned long last_tactile_meas = 0;


void init(){
    hw::init_circuit();
    hw::Fingers::init();
    _motor.init();
    //_led.init();
    _button.init();
}

Button *button(){
    return &_button;
}
Led *led(){
    return &_led;
}
motor_interface::Motor *motor(){
    return &_motor;
}

motor_interface::Motor &motor_ref(){
    return _motor;
}

void measure_tactile(unsigned int Ts) {
    while(millis()-last_tactile_meas < Ts){
        threads.yield();
    }
    hw::Fingers::measure_tactile();
}

float current_force(){
    return hw::Fingers::total_force();
}

}//hw