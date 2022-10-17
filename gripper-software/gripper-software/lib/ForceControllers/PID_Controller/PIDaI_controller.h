#pragma once

#include "Arduino.h"
#include "ForceController.h"


class PID_Controller : public ForceController
{

private:

    float e[2]={0,0};
    float i_k=0, d_k=0;
    float _Kp=5, _Ki=0, _Kd=500, _alpha=-0.9, _Icap=0;
    const float u_max = 50;

    void control_step(float des_force, const time_t t_elapsed, float &error, float &cmd_vel, bool &has_settled) override;
    void control_init() override {
        i_k=0, d_k=0;
    }

public:
    PID_Controller(motor_interface::Motor *motor, hw::Fingers::fingers_t &fingers) : ForceController(motor, fingers) {}
    void set_params(float Kp, float Ki, float Kd, float alpha=0, float i_cap=100);

    void print_params(Print &p);
};
