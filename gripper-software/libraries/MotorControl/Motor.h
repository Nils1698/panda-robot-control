
#pragma once 

#include "MotorCom.h"
#include "MotorSpecs.h"

namespace motor_interface {

constexpr float DEFAULT_SPEED = 100;
enum move_direction_t {
  DIR_CLOSING = true,
  DIR_OPENING = false
};

class Motor : public motor_interface::MotorCom {

  static constexpr float MM_2_USER_DEFINED = 1000; // mm to user defined unis (micrometers)
  static constexpr unsigned long STATE_CHANGE_TIMEOUT = 1000;

  private:
    int32_t position_limit;
    float goto_speed = DEFAULT_SPEED;
    struct force_calib_t {
      float coef_in[2]={1,0};
      float coef_out[2]={1,0};
    };
    force_calib_t force_calib = {{0.04932,0.75363},{0.12927,-1.97526}};

    float current_pos = -1;
    float current_vel = 0;
    int16_t current_torque = 0;

  public:
    Motor();
    void init();

    // ACTIONS
    void go_to(float pos_mm, bool relative=false, bool blocking=false, bool immediate=true);
    void go_to_spd(float pos_mm, float speed, bool relative=false, bool blocking=false, bool immediate=true);
    void set_goto_speed(float spd_mm_s);
    void hold_position();
    inline void open(bool blocking=false){go_to(max_width(), false, blocking);};
    bool target_pos_reached();

    void velocity_control_init(float vel_mm_s=0);
    void velocity_control_set(float vel_mm_s);

    void pos_control_init(float pos_mm=0);
    void pos_control_set(float pos_mm);

    // STATE OPERATIONS
    bool is_connected();
    void enable_operation();
    void disable_operation();
    void disable_voltage();
    void shut_down();
    void quick_stop();
    inline void halt(){change_state(MC3001::MASK_CONTROL_HALT,  MC3001::MASK_CONTROL_HALT);}
    inline void unhalt(){change_state(MC3001::MASK_CONTROL_HALT,  ~MC3001::MASK_CONTROL_HALT);}

    void limit_grip(float min_grip_mm);
    void unlimit_grip();

    // READING
    int16_t read_torque_raw();
    float read_force(move_direction_t direction=DIR_CLOSING);
    int32_t read_raw_position();
    float read_position();    
    float read_velocity();

    // GETTERS
    float get_position();
    float get_velocity();
    int16_t get_torque_raw();

    // HOMING & CONFIGURATION
    bool enable_pos_limits(bool enable);
    inline float max_width(){return (float)position_limit/MM_2_USER_DEFINED;}

    inline void perform_homing(){
        operation_allowed=true;
        perform_homing_in();
        perform_homing_out();
    }

  private:

    void perform_homing_in(/*bool await=true*/);
    void perform_homing_out();

    bool configure_gearing();
    bool configure_homing();
  //   bool configure_pwm_velocity();


  // TEST
  public:
    bool set_gripperFeed(uint32_t feed);
    bool reset_gripperFeed();

};

} // motor
