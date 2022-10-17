
#include "HW_prototype.h"

namespace exec_test {

void inline test_quickstop(){
    float p0,p1;
    hw::motor()->open(true);
    threads.delay(1000);
    hw::motor()->go_to(10);
    threads.delay(300);
    // debug_stream << "STOP!\n";
    p0 = hw::motor()->read_position();
    // debug_stream << "p0: " << p0 << "\n";
    hw::motor()->quick_stop();
    threads.delay(1000);
    p1 = hw::motor()->read_position();
    debug_stream << "p1: " << p1 << " p0: " << p0 << "mm\n";
    info_stream << "Stopping distance: " << p1-p0 << "mm\n";
}

void inline test_stop_time(float speed){

    const int16_t DETECTION_TORQUE = 400; // raw
    const float DETECTION_FORCE = 0.01;

    float init_pos = hw::motor()->read_position();
    float stop_pos = init_pos;
    float timeout_ms = 1.5 * init_pos/speed * 1000;
    hw::motor()->velocity_control_init(-speed);
    unsigned long timeStart = millis();
    while(millis() - timeStart < timeout_ms){
        stop_pos = hw::motor()->read_position(); // < 2.1ms
        float mot_torque = hw::motor()->read_torque_raw(); // < 2.1ms
        hw::measure_tactile(); // < 4ms
        float tact_force = hw::current_force();
        if(tact_force > DETECTION_FORCE){
            break;
        }
        if(fabs(mot_torque) > DETECTION_TORQUE){
            debug_stream<<"Stopped due to motor torque! " << fabs(mot_torque) << "\n";
            break;
        }
        if(millis()-timeStart>1000){
            debug_stream<<"Stopped due to time out! " << millis()-timeStart << "\n";
            break;
        }

    }
    hw::motor()->velocity_control_set(0); // < 2.1ms
    // hw::motor()->hold_position();
    threads.delay(100);
    float end_pos = hw::motor()->read_position(); // < 2.1ms
    signal_stream << end_pos-stop_pos;
}

inline void torque_vs_position(float spd, float min_pos=10, unsigned int hold_time=0){
    hw::motor()->set_goto_speed(motor_interface::DEFAULT_SPEED);
    hw::motor()->open(true);
    hw::motor()->set_goto_speed(spd);

    int dir = -1;
    hw::motor()->go_to(min_pos);
    while(!hw::motor()->target_pos_reached()){
        float mot_torque = hw::motor()->read_torque_raw();
        float pos = hw::motor()->read_position();
        signal_stream << pos << " " << mot_torque << " " << dir << "\n";
        threads.delay(5);
    }

    dir = 0;
    if(hold_time>0){
        hw::motor()->hold_position();
        unsigned long ti = millis();
        while(millis()-ti < hold_time){
            float mot_torque = hw::motor()->read_torque_raw();
            float pos = hw::motor()->read_position();
            signal_stream << pos << " " << mot_torque << " " << dir << "\n";
            threads.delay(5);
        }
    }

    dir = 1;
    hw::motor()->open();
    while(!hw::motor()->target_pos_reached()){
        float mot_torque = hw::motor()->read_torque_raw();
        float pos = hw::motor()->read_position();
        signal_stream << pos << " " << mot_torque << " " << dir << "\n";
        threads.delay(5);
    }
}

inline void force_vs_position(float spd, float min_pos=10, unsigned int hold_time=0){
    hw::motor()->set_goto_speed(motor_interface::DEFAULT_SPEED);
    hw::motor()->open(true);
    hw::motor()->set_goto_speed(spd);

    int dir = -1;
    hw::motor()->go_to(min_pos);
    while(!hw::motor()->target_pos_reached()){
        float mot_torque = hw::motor()->read_force(motor_interface::DIR_CLOSING);
        float pos = hw::motor()->read_position();
        signal_stream << pos << " " << mot_torque << " " << dir << "\n";
        threads.delay(5);
    }

    dir = 0;
    if(hold_time>0){
        hw::motor()->hold_position();
        unsigned long ti = millis();
        while(millis()-ti < hold_time){
            float mot_torque = hw::motor()->read_force(motor_interface::DIR_CLOSING);
            float pos = hw::motor()->read_position();
            signal_stream << pos << " " << mot_torque << " " << dir << "\n";
            threads.delay(5);
        }
    }

    dir = 1;
    hw::motor()->open();
    while(!hw::motor()->target_pos_reached()){
        float mot_torque = hw::motor()->read_force(motor_interface::DIR_OPENING);
        float pos = hw::motor()->read_position();
        signal_stream << pos << " " << mot_torque << " " << dir << "\n";
        threads.delay(5);
    }
}



inline void torque_vs_position_rev(float spd, float min_pos=10, unsigned int hold_time=0){
    hw::motor()->set_goto_speed(motor_interface::DEFAULT_SPEED);
    hw::motor()->go_to(min_pos,false,true);
    hw::motor()->set_goto_speed(spd);

    int dir = 1;
    hw::motor()->open();
    while(!hw::motor()->target_pos_reached()){
        float mot_torque = hw::motor()->read_torque_raw();
        float pos = hw::motor()->read_position();
        signal_stream << pos << " " << mot_torque << " " << dir << "\n";
        threads.delay(5);
    }

    dir = 0;
    if(hold_time>0){
        hw::motor()->hold_position();
        unsigned long ti = millis();
        while(millis()-ti < hold_time){
            float mot_torque = hw::motor()->read_torque_raw();
            float pos = hw::motor()->read_position();
            signal_stream << pos << " " << mot_torque << " " << dir << "\n";
            threads.delay(5);
        }
    }

    dir = -1;
    hw::motor()->go_to(min_pos);
    while(!hw::motor()->target_pos_reached()){
        float mot_torque = hw::motor()->read_torque_raw();
        float pos = hw::motor()->read_position();
        signal_stream << pos << " " << mot_torque << " " << dir << "\n";
        threads.delay(5);
    }
}

inline void print_torque(){
    while(1){
        float mot_torque = hw::motor()->read_torque_raw();
        signal_stream << mot_torque << "\n";
        threads.delay(5);
    }
}

void inline motot_step(float step){
    unsigned long time_init = millis();
    unsigned long time_last = time_init;
    unsigned long time_done = time_init+3000;
    float init_pos = hw::motor()->read_position();
    bool commmanded = false, done = false;
    while(millis() < time_done+200){
        if(!commmanded && millis() - time_init > 200){
            hw::motor()->go_to(step, true);
            commmanded=true;
        }
        else if(commmanded && !done && hw::motor()->target_pos_reached()){
            debug_stream<<"Target reached\n";
            time_done=millis();
            done = true;
        }

        if(millis() - time_last >= 5){
            time_last = millis();
            float pos = hw::motor()->read_position();
            signal_stream << time_last-time_init << " " << pos-init_pos << "\n";
        }
    }
}

} // exec_test