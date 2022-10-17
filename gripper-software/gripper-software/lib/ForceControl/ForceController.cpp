#include "ForceController.h"

#include "Streaming.h"

void ForceController::init_control(float force){
    debug_stream << "init control\n";
    if(interupt_lock.try_lock()){
        stop_flag = false;
        des_force = force;
        _has_settled = false;
        _is_active=true;
        time_init = millis();
        motor->velocity_control_init(); // TODO uses ProfileVelocity  (detection_controller should use this)
        control_init();
        interupt_lock.unlock();
    }
    else{
        error_stream << "Failed to init control, mutex not released!\n";
    }
}

void ForceController::control_loop(std::function<bool()> check_func, bool settle_check){
    debug_stream << "Control loop called\n";
    time_last_step = millis();
    while(1)
    {
        /*Lock block*/{
            Threads::Scope scope(interupt_lock);
            if(stop_flag) break;
        }

        time_t elapsed = millis()-time_last_step;
        if(elapsed >= T_ms){
            if(elapsed > T_ms+10) warning_stream << "Force Control - update time not kept! ("<< elapsed << "ms)\n";
            
            time_last_step = millis();
            
            /*Lock block*/{
                Threads::Scope scope(interupt_lock);
                control_step(des_force, elapsed, force_error, cmd_vel, _has_settled);
                if(settle_check && _has_settled) break;
            }
        if(check_func && !check_func()) break; // check_func may try to acquire interupt_lock itself
        }

        delay(0.1*T_ms); // Give time for other threads to aquire lock
    }
    debug_stream << "Control stopped\n";
    motor->velocity_control_set(0);

    /*Lock block*/{
    Threads::Scope scope(interupt_lock);
    _is_active=false;
    }
}

void ForceController::run(float force, std::function<bool()> chk_func){
    if(is_active()){
        warning_stream << "Control already active (ForceController::run)\n";
        return;
    }
    init_control(force);
    control_loop(chk_func, false);
}

void ForceController::run_settled(float force){
    if(is_active()){
        warning_stream << "Control already active (ForceController::run)\n";
        return;
    }
    init_control(force);
    control_loop(nullptr, true);
}


void ForceController::set_force(float force){
    if(force <= FORCE_LIMIT){
        Threads::Scope scope(interupt_lock);
        des_force = force;
        _has_settled=false;
    }else{
        warning_stream << "Set force : Exceeds limit! ("<<FORCE_LIMIT<<"N)\n";
    }
}

float ForceController::get_desired_force(){
    Threads::Scope scope(interupt_lock);
    return des_force;
}

float ForceController::get_cmd_vel(){
    Threads::Scope scope(interupt_lock);
    return cmd_vel;
}

void ForceController::stop(){
    Threads::Scope scope(interupt_lock);
    stop_flag = true;
}

bool ForceController::has_settled(){
    Threads::Scope scope(interupt_lock);
    return _has_settled;
}

float ForceController::get_error(){
    Threads::Scope scope(interupt_lock);
    return force_error;
}

float ForceController::get_rel_error(){
    Threads::Scope scope(interupt_lock);
    return force_error/des_force;
}

bool ForceController::is_active(){
    Threads::Scope scope(interupt_lock);
    return _is_active;
}