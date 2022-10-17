
#include "force_controller.h"
#include "HW_prototype.h"

namespace force_control {

bool controller_running;
bool stop_flag;
constexpr unsigned int T_ms = 20;
unsigned long time_last_step;
unsigned long time_init;
Threads::Mutex interupt_lock;
float init_pos;
float des_force=0;
float des_pos=0;
bool _has_settled=false;
constexpr float FORCE_LIMIT = 20;

float ku[2]={0,0}, ke[3]={0,0,0};

exec::Executer control_executor;

// constexpr float MAX_SPEED = 1; // mm/s
// constexpr float MAX_POS_CHANGE = MAX_SPEED*T_ms/1000;
// inline void control_step(){
//     // unsigned long t0 = millis();
//     hw::measure_tactile();
//     // unsigned long t1 = millis();
//     // debug_stream << "meas time: " << t1-t0 << "\n";
//     // return;
    
//     float force = hw::current_force();
//     debug_stream << "Force: " << force << " err:" << force-des_force << "\n";
//     float new_pos = init_pos+(force-des_force)*5;
//     float pos_change = new_pos-des_pos;
//     pos_change = pos_change>0 ? min(pos_change, MAX_POS_CHANGE) : max(pos_change, -MAX_POS_CHANGE);
//     des_pos += pos_change;
//     // debug_stream << "send pos: " << des_pos << "\n";
//     hw::motor()->pos_control_set(des_pos);
// }

// float uold[2] = {0,0};
// float yold = 0;
// constexpr float a[] = {7700, -15403.6, 7703.57};
// constexpr float a[] = {7705, -15405, 7700};
// constexpr float a[] = {7720, -15420, 7700};
// constexpr float a[] = {7740, -15440, 7700};
// constexpr float a[] = {7780, -15480, 7700};
// constexpr float a[] = {7860, -15560, 7700};

// constexpr float a[] = {386000.0/50, -770999.0/50, 385000.0/50}; // I increased
// constexpr float a[] = {8020, -15719.9, 7700}; // P:320 I:5 D:154
// constexpr float a[] = {8070, -1639.9, 7700};  // P:1000 I:50 D:154

// inline void control_step(){ // Anders 21. nov 2021
//     const float W_TO_MM = 2.18/60;
//     float c = hw::circII.measure_conductance(6);
//     // des_force = 3 + 2*sin(millis()/1000);
//     float c_des = (-121.1*des_force*des_force + 4375*des_force)*1e-6;
//     float e = c - c_des;
//     // debug_stream << "err: " << e*1000000 << "\n";
//     // float e = c - 
//     // debug_stream << "c:" << c*1000000 << "\n";

//     float y = a[0]*uold[1] + a[1]*uold[0] + a[2]*e + yold;

//     // debug_stream << "y:" << y*W_TO_MM << "\n";
//     signal_stream << c_des*1000000 << " " << c*1000000 << " " << e*1000000 << " " << y*W_TO_MM << "\n";
//     // hw::motor()->velocity_control_set(y*W_TO_MM);

//     // // newly added:
//     // yold = y;
//     // uold[1] = uold[0];
//     // uold[0] = e;
// }


// inline void control_step(){ // Anders 21. nov 2021, corrected
//     const float W_TO_MM = 2.18/60;
//     float c = hw::circII.measure_conductance(6);
//     // des_force = 3 + 2*sin(millis()/1000);
//     float c_des = (-121.1*des_force*des_force + 4375*des_force)*1e-6;
//     float e = c - c_des;
//     // debug_stream << "err: " << e*1000000 << "\n";
//     // float e = c - 
//     // debug_stream << "c:" << c*1000000 << "\n";

//     float y = a[0]*e + a[1]*uold[1] + a[2]*uold[0] + yold; // Works but is wrong

//     // debug_stream << "y:" << y*W_TO_MM << "\n";
//     signal_stream << c_des*1000000 << " " << c*1000000 << " " << e*1000000 << " " << y*W_TO_MM << "\n";
//     // hw::motor()->velocity_control_set(y*W_TO_MM);

//     // // newly added:
//     yold = y;
//     uold[1] = uold[0];
//     uold[0] = e;
// }


// float eold[2] = {0,0};
// float yold[2] = {0,0};
// // constexpr float a[] = {15720.05, -30799.9, 15080.05};
// constexpr float a[] = {8020.05, -15719.95, 7700.0};

// inline void control_step(){ // Anders 21. nov 2021, corrected2
//     const float W_TO_MM = 2.18/60;
//     float c = hw::circII.measure_conductance(6);
//     // des_force = 3 + 2*sin(millis()/1000);
//     float c_des = (-121.1*des_force*des_force + 4375*des_force)*1e-6;
//     float e = c - c_des;
//     // debug_stream << "err: " << e*1000000 << "\n";
//     // float e = c - 
//     // debug_stream << "c:" << c*1000000 << "\n";

//     float y = a[0]*e + a[1]*eold[0] + a[2]*eold[1] + yold[0];

//     // debug_stream << "y:" << y*W_TO_MM << "\n";
//     signal_stream << c_des*1000000 << " " << c*1000000 << " " << e*1000000 << " " << _FLOAT(y*W_TO_MM,4) << "\n";
//     // hw::motor()->velocity_control_set(y*W_TO_MM);

//     // // newly added:
//     yold[1] = yold[0];
//     yold[0] = y;
//     eold[1] = eold[0];
//     eold[0] = e;
// }


// float e[3] = {0,0,0};
// float u[3] = {0,0,0};
// inline void control_step(){
//     // https://www.scilab.org/discrete-time-pid-controller-implementation
//     constexpr float Kp = 7700;
//     constexpr float Ki = 0.01;
//     constexpr float Kd = 154;
//     constexpr float Ts = T_ms;
//     constexpr float N = 1;
//     constexpr float b[] = {
//         Kp*(1+N*Ts) + Ki*Ts*(1+N*Ts) + Kd*N,
//         -(Kp*(2+N*Ts) + Ki*Ts + 2*Kd*N),
//         Kp + Kd*N
//     };
//     constexpr float a[] = {
//         1+N*Ts,
//         -(2+N*Ts),
//         1
//     };

//     const float g = hw::circII.measure_conductance(6);
//     const float g_des = (-121.1*des_force*des_force + 4375*des_force)*1e-6;
//     e[0] = g_des - g;

//     u[0] = - a[1]/a[0]*u[1] - a[2]/a[0]*u[2] + b[0]/a[0]*e[0] + b[1]/a[0]*e[1] + b[2]/a[0]*e[2];

//     constexpr float W_TO_MM = 2.18/60;
//     signal_stream << g_des*1000000 << " " << g*1000000 << " " << e[0]*1000000 << " " << u[0]*W_TO_MM << "\n";
//     hw::motor()->velocity_control_set(-u[0]*W_TO_MM);

//     u[2]=u[1];
//     u[1]=u[0];
//     e[2]=e[1];
//     e[1]=e[0];
// }


// inline void control_step(){ // F2221 foam fingers
//     hw::measure_tactile();
//     float force = hw::current_force();
//     float e = (force - des_force);
//     float y = 10*e;

//     // signal_stream << force << " " << e << " " << y << "\n";
//     hw::motor()->velocity_control_set(y);
//     hw::Fingers::F[0]->TactileSensor.print(false);
//     hw::Fingers::F[1]->TactileSensor.print();

// } 



// float e[3] = {0,0,0};
// float u[3] = {0,0,0};
// inline void control_step(){ // Foam fingers
//     // https://www.scilab.org/discrete-time-pid-controller-implementation
//     // constexpr float Kp = 10;
//     // constexpr float Ki = 0;
//     // constexpr float Kd = 0;
//     // constexpr float Ts = T_ms;
//     // constexpr float N = 1;
//     // constexpr float b[] = {
//     //     Kp*(1+N*Ts) + Ki*Ts*(1+N*Ts) + Kd*N,
//     //     -(Kp*(2+N*Ts) + Ki*Ts + 2*Kd*N),
//     //     Kp + Kd*N
//     // };
//     // constexpr float a[] = {
//     //     1+N*Ts,
//     //     -(2+N*Ts),
//     //     1
//     // };

//     hw::measure_tactile();
//     float force = hw::current_force();
//     e[0] = des_force - force;

//     // u[0] = - a[1]/a[0]*u[1] - a[2]/a[0]*u[2] + b[0]/a[0]*e[0] + b[1]/a[0]*e[1] + b[2]/a[0]*e[2];
//     u[0] = - ku[0]*u[1] - ku[1]*u[2] + ke[0]*e[0] + ke[1]*e[1] + ke[2]*e[2];

//     signal_stream << force << " " << e[0] << " " << -u[0] << "\n";
//     hw::motor()->velocity_control_set(-u[0]);

//     u[2]=u[1];
//     u[1]=u[0];
//     e[2]=e[1];
//     e[1]=e[0];
// }


// https://engineering.stackexchange.com/questions/26537/what-is-a-definitive-discrete-pid-controller-equation
float e[2]={0,0};
float i_k=0, d_k=0;
float _Kp=5, _Ki=0, _Kd=500, _alpha=-0.9, _Icap=0;
constexpr float u_max = 50;
void control_step(){  
    hw::measure_tactile();
    const float force = hw::current_force();
    e[0] = - (des_force - force);

    i_k = max(min(i_k + T_ms*(e[0]+e[1])/2, _Icap), -_Icap);
    // float d_k = (e[0]-e[1])/T_ms; // alpha=1
    d_k = (1+_alpha)*(e[0]-e[1])/T_ms - _alpha*d_k;
    float u = max(min( _Kp*e[0] + _Ki*i_k + _Kd*d_k, u_max), -u_max);

    hw::motor()->velocity_control_set(u);

    // debug_stream << "e: "<<e[0]<< ", i: "<<i_k << ", d: "<<d_k << ", u: " << u << "\n";
    // signal_stream << _FLOAT(e[0],4) << " " << _FLOAT(_Ki*i_k,4) << " "  << _FLOAT(_Kd*d_k,4) << " "  << _FLOAT(u,4) << "\n";
#ifdef USE_OPT_SENSOR
    signal_stream << _FLOAT(des_force,4) << " " << _FLOAT(force,4) << " " << _FLOAT(u,4) << "\n";
#endif
    signal_stream << _FLOAT(des_force,4) << " " << _FLOAT(force,4) << " " << _FLOAT(u,4) << "\n";

// #ifdef CONTROLLER_STREAM
//     hw::Fingers::F[0]->TactileSensor.print(false);
//     hw::Fingers::F[1]->TactileSensor.print();
// #endif

    if(!_has_settled && abs(e[0]/des_force) < 0.05){
        _has_settled=true;
        // debug_stream << "Force controller has settled\n";
    }
    // _has_settled = _has_settled || abs(e[0]/des_force) < 0.02; // error within 2%

    e[1]=e[0];
}
void set_PID(float Kp, float Ki, float Kd, float alpha, float i_cap){
    if(control_executor.is_executing()){
        warning_stream << "Stop control before setting PID values!\n";
        return;
    }
    _Kp=Kp;
    _Ki=Ki;
    _Kd=Kd;
    _alpha=alpha;
    _Icap = i_cap;
}

void control_loop(){
    while(1)
    {
        if(millis()-time_last_step >= T_ms){
            if(millis()-time_last_step > T_ms+10){
                warning_stream << "Force Control - update time not kept!\n";
            }
            time_last_step = millis();
            control_step();
        }

        Threads::Scope scope(interupt_lock);
        if(stop_flag) break;
    }
    debug_stream << "Control stopped\n";
    hw::motor()->velocity_control_set(0);
    // hw::motor()->hold_position();
    // TODO stop motor
}

void init_control(float force){
    stop_flag = false;
    time_last_step = 0;
    set_force(force);
    time_init = millis();
    hw::motor()->velocity_control_init();

    // hw::motor()->pos_control_init();
    // init_pos = hw::motor()->read_position();
    // des_pos = init_pos;
    // debug_stream << "Controler - init_pos = " << init_pos << "\n";
}

void start_control(float force){
    if(control_executor.is_executing()){
        warning_stream << "Control already running!\n";
        return;
    }
    init_control(force);
    control_executor.execute(control_loop);
}

void set_force(float force){
    if(force <= FORCE_LIMIT){
        Threads::Scope scope(interupt_lock);
        des_force = force;
        _has_settled=false;
    }else{
        warning_stream << "Set force : Exceeds limit! ("<<FORCE_LIMIT<<"N)\n";
    }
}

float get_desired_force(){
    Threads::Scope scope(interupt_lock);
    return des_force;
}

void stop_control(){
    Threads::Scope scope(interupt_lock);
    stop_flag = true;
}

bool has_settled(){
    Threads::Scope scope(interupt_lock);
    return _has_settled;
}

float get_error(){
    Threads::Scope scope(interupt_lock);
    return e[0];
}
float get_rel_error(){
    Threads::Scope scope(interupt_lock);
    return e[0]/des_force;
}


bool is_active(){
    return control_executor.is_executing();
}

// void set_PID(float Kp, float Ki, float Kd, float N){
//     if(control_executor.is_executing()){
//         warning_stream << "Stop control before setting PID values!\n";
//         return;
//     }
//     constexpr float Ts = T_ms;
//     const float a0 = (1+N*Ts);
//     ke[0] = Kp + Ki*Ts + Kd*N/a0;
//     ke[1] = -((Kp*(2+N*Ts) + Ki*Ts + 2*Kd*N)/a0);
//     ke[2] = (Kp + Kd*N)/a0;
//     ku[0] = -(2+N*Ts)/a0;
//     ku[1] = 1/a0;
//     debug_stream << ku[0] << " " << ku[1] << " " << ke[0] << " " << ke[1] << " " << ke[2] << "\n";
// }

void kill_control(){
    control_executor.kill_execution();
}

} // force_control


