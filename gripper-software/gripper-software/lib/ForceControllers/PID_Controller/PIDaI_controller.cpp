#include "PIDaI_controller.h"

#include "fingers.h"

// https://engineering.stackexchange.com/questions/26537/what-is-a-definitive-discrete-pid-controller-equation
void PID_Controller::control_step(float des_force, const time_t t_elapsed, float &error, float &cmd_vel, bool &has_settled){  
    hw::Fingers::measure_tactile();    
    const float force = hw::Fingers::total_force();
    // hw::measure_tactile();
    // const float force = hw::current_force();
    e[0] = - (des_force - force);
    error = e[0];

    i_k = max(min(i_k + t_elapsed*(e[0]+e[1])/2, _Icap), -_Icap);
    // float d_k = (e[0]-e[1])/T_ms; // alpha=1
    d_k = (1+_alpha)*(e[0]-e[1])/t_elapsed - _alpha*d_k;
    //debug_stream << _Kp*e[0] << " + " << _Ki*i_k << " + " << _Kd*d_k << "\n";
    cmd_vel = max(min( _Kp*e[0] + _Ki*i_k + _Kd*d_k, u_max), -u_max);

    motor->velocity_control_set(cmd_vel);
    //hw::motor()->velocity_control_set(u);

    // debug_stream << "e: "<<e[0]<< ", i: "<<i_k << ", d: "<<d_k << ", u: " << u << "\n";
    // signal_stream << _FLOAT(e[0],4) << " " << _FLOAT(_Ki*i_k,4) << " "  << _FLOAT(_Kd*d_k,4) << " "  << _FLOAT(u,4) << "\n";
    
    // signal_stream << _FLOAT(des_force,4) << " " << _FLOAT(force,4) << " " << _FLOAT(u,4) << "\n";

// #ifdef CONTROLLER_STREAM
//     hw::Fingers::F[0]->TactileSensor.printTo(signal_stream,false);
//     hw::Fingers::F[1]->TactileSensor.printTo(signal_stream);
// #endif

    // TODO proper computation on whether control has reach it's goal (vel should be 0 and des_force=0 should be possible)
    if(!has_settled && abs(e[0]/des_force) < 0.05){
        has_settled=true;
        // debug_stream << "Force controller has settled\n";
    }
    // _has_settled = _has_settled || abs(e[0]/des_force) < 0.02; // error within 2%

    e[1]=e[0];
}
void PID_Controller::set_params(float Kp, float Ki, float Kd, float alpha, float i_cap){
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

void PID_Controller::print_params(Print &p){
    p   << "Kp: " << _Kp << " "
        << "Ki: " << _Ki << " "
        << "Kd: " << _Kd << " "
        << "alpha: " << _alpha << " "
        << "Icap: " << _Icap << "\n";
}