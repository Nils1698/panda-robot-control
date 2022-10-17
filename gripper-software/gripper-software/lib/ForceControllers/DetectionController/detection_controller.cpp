#include "detection_controller.h"

#include "fingers.h"

// https://engineering.stackexchange.com/questions/26537/what-is-a-definitive-discrete-pid-controller-equation
void Detect_Controller::control_step(float des_force, const time_t t_elapsed, float &error, float &cmd_vel, bool &has_settled){  
    // hw::measure_tactile();
    // const float force = hw::current_force();
    hw::Fingers::measure_tactile();    
    const float force = hw::Fingers::total_force();
    if(force < detection_force){
        // debug_stream << "fixed vel\n";
        e[1]=0;
        i_k=0;
        d_k=0;
        has_settled=false;
        cmd_vel = -dspeed;
        motor->velocity_control_set(cmd_vel); // ProfileVelocity
        dspeed = max(10, min(50, dspeed+0.05*t_elapsed));
    }
    else
    {
        // debug_stream << "force control\n";
        e[0] = - (des_force - force);
        error = e[0];

        i_k = max(min(i_k + t_elapsed*(e[0]+e[1])/2, _Icap), -_Icap);
        d_k = (1+_alpha)*(e[0]-e[1])/t_elapsed - _alpha*d_k;
        cmd_vel = max(min( _Kp*e[0] + _Ki*i_k + _Kd*d_k, u_max), -u_max);

        motor->velocity_control_set(cmd_vel);

        // TODO proper computation on whether control has reach it's goal (vel should be 0 and des_force=0 should be possible)
        if(!has_settled && abs(e[0]/des_force) < 0.05){
            has_settled=true;
            // debug_stream << "Force controller has settled\n";
        }
        // _has_settled = _has_settled || abs(e[0]/des_force) < 0.02; // error within 2%

        e[1]=e[0];

        dspeed=detection_speed;
    }

}
void Detect_Controller::set_params(float Kp, float Ki, float Kd, float alpha, float i_cap){
    if(is_active()){
        warning_stream << "Stop control before setting PID values!\n";
        return;
    }
    _Kp=Kp;
    _Ki=Ki;
    _Kd=Kd;
    _alpha=alpha;
    _Icap = i_cap;
}

void Detect_Controller::print_params(Print &p){
    p   << "Kp: " << _Kp << " "
        << "Ki: " << _Ki << " "
        << "Kd: " << _Kd << " "
        << "alpha: " << _alpha << " "
        << "Icap: " << _Icap << "\n";
}