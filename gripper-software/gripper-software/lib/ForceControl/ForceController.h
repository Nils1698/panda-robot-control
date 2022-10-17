#pragma once

#include "Arduino.h"
#include "TeensyThreads.h"
#include "functional"

#include "Motor.h"
#include "fingers.h"

class ForceController
{
public:
    const float FORCE_LIMIT = 20;

private:
    // Variables shared between threads
    bool stop_flag;
    float des_force=0;
    float force_error=0;
    float cmd_vel=0;
    bool _has_settled=false;
    bool _is_active = false;
    //
    Threads::Mutex interupt_lock;

    const time_t T_ms = 20;
    unsigned long time_last_step;
    unsigned long time_init;

    void init_control(float force);
    void control_loop(std::function<bool()> chk_func, bool settle_check);
    virtual void control_step(float des_force, const time_t t_elapsed, float &error, float &cmd_vel, bool &has_settled)=0;
    virtual void control_init()=0;

protected:    
    motor_interface::Motor * const motor;
    const hw::Fingers::fingers_t & fingers;

public:
    ForceController(motor_interface::Motor *motor, hw::Fingers::fingers_t &fingers) : motor(motor), fingers(fingers) {}
    void run(float force=0, std::function<bool()> chk_func=nullptr);
    void run_settled(float force);
    void set_force(float force);
    bool has_settled();
    float get_error();
    float get_rel_error();
    float get_desired_force();
    float get_cmd_vel();

    bool is_active();
    void stop();

    virtual void print_params(Print &p) = 0;
};
