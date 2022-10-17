
#include "Motor.h"
#include "UserStream.h"

namespace motor_interface {

//constexpr uint16_t HOMING_TORQUE_LIMIT = 250; // from 400 (2250S)
//constexpr float HOMING_SPEED = 20; // from 50 (2250S)
//constexpr float MAX_PROFILE_VEL = 150;

Motor::Motor(){}

void Motor::init(){
  MotorCom::init();

  if(connection_status==CONNECTION_DOWN){
    return;
  }

  get_parameter<int32_t>(0x607D, 0x02, position_limit); // position max limit TODO

  // for debugging
  debug_stream << "raw pos: " << read_raw_position() << "\n";
  debug_stream << "max limit: " << position_limit << "\n";
  float pos = read_position();
  if(pos < 1 || pos > 200){
    info_stream << "Homing of gripper needed\n";
    operation_allowed=false;
  }else{
    operation_allowed=true;
  }
  
  // TODO discard all old movement jobs

  // get_parameter<uint32_t>(0x6081, 0x00, current_profile_speed);
  
  if(configure_gearing()){
    debug_stream << "Successfully configured gearing\n";
  }else{
    error_stream << "Failed to configure gearing\n";
  }

  if(configure_homing()){
    debug_stream << "Successfully configured homing\n";
  }else{
    error_stream << "Failed to configure homing\n";
  }

  // configure halt and quick stop
  bool success = true;
  success = success && set_parameter<int16_t>(0x605A,0x00, 6); // Quick stop option code -> 'Slow down on quick stop ramp and stay in Quick Stop Active'
  success = success && set_parameter<uint32_t>(0x6085,0x00, 5000); // Quick stop decelleration [1/s^2]
  success = success && set_parameter<int16_t>(0x605D,0x00, 1); // Set halt action to slow down ramp
  success = success && set_operation_mode(0b1,0b1); // Reset set-point when changing mode
  if(!success){
    error_stream << "Failed to set halt/quickstop configuration!\n";
  }

  set_control_mode(MC3001::Inactive);
  shut_down();
  threads.delay(10);
  enable_operation();
  threads.delay(10);
}

bool Motor::is_connected(){
  return connection_status==CONNECTION_OK;
}

void Motor::perform_homing_in(){
  const unsigned long HOMING_TIMEOUT = 10*1000; // 10s

  debug_stream << "Perform homing\n";
  set_control_mode(MC3001::Homing);
  enable_operation();
  if(begin_operation()){

    resetStatusFlag();
    if(awaitStatus(MC3001::MASK_HOMING_REACHED|MC3001::MASK_HOMING_STOPPED, 
                  MC3001::MASK_HOMING_REACHED|MC3001::MASK_HOMING_STOPPED, HOMING_TIMEOUT)){ // TODO check for homing error as well
      debug_stream << "Homing done\n";
    }else{
      debug_stream << "Homing timed out!\n";  
      // TODO handle time out
    }

  }
}

void Motor::perform_homing_out(){
  const float MAX_STROKE_MM = 200; // mm   // TODO 
  
  // Set force limits
  bool success = true;
  uint16_t old_torque_lim_pos, old_torque_lim_neg;
  success = success && get_parameter<uint16_t>(0x60E0,0x00, old_torque_lim_pos);
  success = success && get_parameter<uint16_t>(0x60E1,0x00, old_torque_lim_neg);
  if(success){
    success = success && set_parameter<uint16_t>(0x60E0,0x00, HOMING_TORQUE_LIMIT ); // Positive torque limit
    success = success && set_parameter<uint16_t>(0x60E1,0x00, HOMING_TORQUE_LIMIT ); // Negative torque limit
  }
  if(!success){
    warning_stream << "Setting force limits failed\n";
    return;
  }

  enable_pos_limits(false);
  enable_operation();
  velocity_control_init(HOMING_SPEED);
  threads.delay(500); // wait to not detect initial spike in torque when accelerating

  unsigned long timeout = (MAX_STROKE_MM/HOMING_SPEED + 1.0)*1000;
  unsigned long timeStart = millis();
  while(millis() - timeStart < timeout){
    if(abs(read_torque_raw()) >= HOMING_TORQUE_LIMIT-10){
      debug_stream << "homing limit detected\n";
      break;
    }
    threads.yield();
  }
  velocity_control_set(0);

  int32_t act_pos;
  get_parameter<int32_t>(0x6064, 0x00, act_pos); // Actual position
  debug_stream << "max pos: " << act_pos << "\n";
  go_to(-2,true,true);
  position_limit = act_pos-2000; // subtract 2mm
  set_parameter<int32_t>(0x607D,0x02, position_limit); // Set Software position limit max
  enable_pos_limits(true);
  success = success && set_parameter<uint16_t>(0x60E0,0x00, old_torque_lim_pos ); // Positive torque limit (1000 = rated torque = 26.2mNm)
  success = success && set_parameter<uint16_t>(0x60E1,0x00, old_torque_lim_neg ); // Negative torque limit
  if(!success){
    warning_stream << "Failed to reset force limits\n";
  }
}

void Motor::go_to(float pos_mm, bool relative, bool blocking, bool immediate){
  go_to_spd(pos_mm, goto_speed, relative, blocking, immediate);
}

void Motor::go_to_spd(float pos_mm, float speed, bool relative, bool blocking, bool immediate){
  // TODO check pos validity
  
  set_control_mode(MC3001::ProfilePosition);
  set_parameter<uint32_t>(0x6081,0x00, speed * MM_2_USER_DEFINED); // Profile velocity
  set_parameter<int32_t>(0x607A,0x00, (int32_t)(pos_mm * MM_2_USER_DEFINED) ); // Set target pos

  auto options = getControlword();
  if(immediate) options |= (1<<5);
  else          options &= ~(1<<5);    

  if(relative) options |= (1<<6);
  else         options &= ~(1<<6);
  
  if( begin_operation(options) ){

    awaitStatus(MC3001::MASK_PP_TARGET_REACHED, ~MC3001::MASK_PP_TARGET_REACHED, 100);
    resetStatusFlag();

    if(blocking){
      const unsigned long TIMEOUT = 20*1000; // 20s  // TODO contant!
      if(!awaitStatus(MC3001::MASK_PP_TARGET_REACHED, MC3001::MASK_PP_TARGET_REACHED, TIMEOUT)){
        
        warning_stream << "Positioning timed out!\n";
        // TODO handle time out
      }else debug_stream << "Goto done\n";
    }
  }
}

bool Motor::target_pos_reached(){
  return checkFlaggedStatus(MC3001::MASK_PP_TARGET_REACHED, MC3001::MASK_PP_TARGET_REACHED);
}

void Motor::set_goto_speed(float spd_mm_s){
  goto_speed = spd_mm_s;
}

void Motor::hold_position(){
  go_to(0,true,true,true);
}

void Motor::velocity_control_init(float vel_mm_s)
{
  set_control_mode(MC3001::ProfileVelocity);
  set_parameter<uint32_t>(0x6081,0x00, MAX_PROFILE_VEL*MM_2_USER_DEFINED); // Set profile velocity
  if(vel_mm_s != 0) velocity_control_set(vel_mm_s);
  if( begin_operation() ){
    resetStatusFlag();
  }
}

void Motor::velocity_control_set(float vel_mm_s)
{
  set_parameter<int32_t>(0x60FF,0x00, (int32_t)(vel_mm_s * MM_2_USER_DEFINED) ); // Set target velocity
}

void Motor::pos_control_init(float pos_mm){
  set_control_mode(MC3001::CSPosition);
  if(pos_mm != 0) pos_control_set(pos_mm);
  if( begin_operation() ){
    resetStatusFlag();
  }
}
void Motor::pos_control_set(float pos_mm)
{
  set_parameter<int32_t>(0x607A,0x00, (int32_t)(pos_mm * MM_2_USER_DEFINED) ); // Set target position
}

void Motor::enable_operation(){
  change_state(0b1111, 0b1111);
  if(!awaitStatus(MC3001::STATE_MASK1, MC3001::OperationEnabled, STATE_CHANGE_TIMEOUT)){
    warning_stream << "Enable operation failed\n";
  }
}
void Motor::disable_operation(){
  change_state(0b1111, 0b0111);
  if(!awaitStatus(MC3001::STATE_MASK1, MC3001::SwitchedOn, STATE_CHANGE_TIMEOUT)){
    warning_stream << "Disable operation failed\n";
  }
}
void Motor::disable_voltage(){
  change_state(0b0010, 0b0000);
  if(!awaitStatus(MC3001::STATE_MASK2, MC3001::SwitchOnDisabled, STATE_CHANGE_TIMEOUT)){
    warning_stream << "Disable voltage failed\n";
  }
}
void Motor::shut_down(){
  change_state(0b0111, 0b0110);
  if(!awaitStatus(MC3001::STATE_MASK1, MC3001::ReadyToSwitchOn, STATE_CHANGE_TIMEOUT)){
    warning_stream << "Shut down failed\n";
  }
}

void Motor::quick_stop(){
  change_state(MC3001::MASK_CONTROL_QUICKSTOP, ~MC3001::MASK_CONTROL_QUICKSTOP);
  if(!awaitStatus(MC3001::QuickStopActive, MC3001::QuickStopActive, STATE_CHANGE_TIMEOUT)){
      warning_stream << "Quickstop timed out\n";
    }
}

bool Motor::enable_pos_limits(bool enable){
  return set_operation_mode(0b10, enable ? 0b10 : 0b00);
}

// float Motor::read_force(){
//   int16_t act_torque;
//   get_parameter<int16_t>(0x6077, 0x00, act_torque);  
//   return TORQUE_FACTOR * act_torque;  
// }


int16_t Motor::read_torque_raw(){
  int16_t act_torque;
  get_parameter<int16_t>(0x6077, 0x00, act_torque);  
  current_torque = act_torque;
  return act_torque;
}
int16_t Motor::get_torque_raw(){
  return current_torque;
}

float Motor::read_force(move_direction_t direction){
  float torque = (float)read_torque_raw();
  if(direction==DIR_CLOSING){
    return force_calib.coef_in[0]*torque+force_calib.coef_in[1];
  }else{
    return force_calib.coef_out[0]*torque+force_calib.coef_out[1];
  }
}

int32_t Motor::read_raw_position(){
  int32_t act_pos=0;
  get_parameter<int32_t>(0x6064, 0x00, act_pos);
  return act_pos;
}
float Motor::read_position(){
  int32_t act_pos;
  if(get_parameter<int32_t>(0x6064, 0x00, act_pos)){
    // debug_stream << "act_pos: " << act_pos << "\n";
    current_pos = (float)(act_pos)/MM_2_USER_DEFINED;
    return current_pos;
  }else{
    return -1;
  }
}
float Motor::get_position(){
  return current_pos;
}

float Motor::read_velocity(){
  int32_t act_vel;
  if(get_parameter<int32_t>(0x606C, 0x00, act_vel)){
    current_vel=  (float)(act_vel)/MM_2_USER_DEFINED; // um/s -> mm/s
    return current_vel;
  }else{
    return -1;
  }
}
float Motor::get_velocity(){
  return current_vel;
}

void Motor::limit_grip(float min_grip_mm){
  if(min_grip_mm<0) return;
  set_parameter<int32_t>(0x607D,0x01, min_grip_mm*MM_2_USER_DEFINED); // Set Lower software position limit
}
void Motor::unlimit_grip(){
  limit_grip(0);
}


bool Motor::configure_homing(){
  bool success = true;
  success = success && set_operation_mode(0b110000, 0b110000); // bit 4: Ignore position limits, bit 5: Use homing torque limits

  success = success && set_parameter<int8_t>(0x6098,0x00, -3); // Homing method
  
  success = success && set_parameter<uint16_t>(0x2350,0x00, HOMING_TORQUE_LIMIT); // Positive homing torque limit (1000 = rated torque = 26.2mNm)
  success = success && set_parameter<uint16_t>(0x2351,0x00, HOMING_TORQUE_LIMIT); // Negative homing torque limit

//  (int32_t)(vel_mm_s * 1000)
  success = success && set_parameter<uint32_t>(0x6099,0x01, HOMING_SPEED*MM_2_USER_DEFINED); // Homing speed (switch)
  success = success && set_parameter<uint32_t>(0x6099,0x02, HOMING_SPEED*MM_2_USER_DEFINED); // Homing speed (zero)

  success = success && set_parameter<uint32_t>(0x609A,0x00, HOMING_ACC*MM_2_USER_DEFINED); // Homing acceleration

  // constexpr int32_t STROKE = 90*1000; // 90mm  
  success = success && set_parameter<int32_t>(0x607C,0x00, HOMING_OFFSET*MM_2_USER_DEFINED); // Homing offset
  success = success && set_parameter<int32_t>(0x607D,0x01,      0); // Set Software position limit min

  return success;
}

bool Motor::set_gripperFeed(uint32_t feed){
  bool success = true;
  success = success && set_parameter<uint32_t>(0x6092, 0x01, feed); // Feed in micrometers (gripper gearing rev->linear)
  success = success && set_parameter<uint32_t>(0x6092, 0x02, 1); //  pr gear shaft rev
  return success;
}

bool Motor::reset_gripperFeed(){
  bool success = true;
  // Set to  1 mm / shaft rev
  success = success && set_parameter<uint32_t>(0x6092, 0x01, 1*MM_2_USER_DEFINED); // Feed in micrometers (gripper gearing rev->linear)
  success = success && set_parameter<uint32_t>(0x6092, 0x02, 1); //  pr gear shaft rev
  return success;
}


bool Motor::configure_gearing(){
  bool success = true;

  success = success && set_parameter<uint32_t>(0x6091, 0x01, GEARHEAD_RATIO); // Gear ratio, motor shaft
  success = success && set_parameter<uint32_t>(0x6091, 0x02, 1); //  drive shaft

  // success = success && set_parameter<uint32_t>(0x6092, 0x01, 96154); // Feed in micrometers (gripper gearing rev->linear)
  success = success && set_parameter<uint32_t>(0x6092, 0x01, GRIPPER_GEARING); // Feed in micrometers (gripper gearing rev->linear)
  success = success && set_parameter<uint32_t>(0x6092, 0x02, 1); //  pr gear shaft rev

  success = success && set_parameter<uint32_t>(0x6096,0x01,1); // Velocity factor, numerator
  success = success && set_parameter<uint32_t>(0x6096,0x02,60); // Divisor   (rev/min -> rev/s)
  
  success = success && set_parameter<uint32_t>(0x6080,0x00,200000); // Speed limit

  success = success && set_parameter<uint16_t>(0x60E0,0x00, 1000 ); // Positive torque limit (1000 = rated torque = 26.2mNm)
  success = success && set_parameter<uint16_t>(0x60E1,0x00, 1000 ); // Negative torque limit

  // 3216W: (from nothing with 2250S)
  success = success && set_parameter<uint32_t>(0x6083,0x00, 30000 ); // Profile acceleration
  success = success && set_parameter<uint32_t>(0x6084,0x00, 30000 ); // Profile Decceleration

  // Configuring errors //
  success = success && set_parameter<uint32_t>(0x6065,0x00, 1*MM_2_USER_DEFINED ); // FollowingError Window
  success = success && set_parameter<uint16_t>(0x6066,0x00, 1*MM_2_USER_DEFINED ); // FollowingError Time Out (until error report)
  success = success && set_parameter<uint32_t>(0x6067,0x00, 50 ); // Position Window
  success = success && set_parameter<uint16_t>(0x6068,0x00, 48 ); // Position Window Time (until target achived report)


  // success = success && set_parameter<uint16_t>(0x2344,0x03, 65535 ); // Velocity Deviation Threshold
  success = success && set_parameter<uint16_t>(0x2344,0x04, 100 ); // Velocity Deviation Time
  success = success && set_parameter<uint32_t>(0x2344,0x05, 30000 ); // Velocity Warning Threshold // TODO gives DynamicError

  return success;
}

// bool Motor::configure_pwm_velocity(){
//   bool success = true;

//   success = success && set_parameter<uint8_t>(0x2317, 0x01, 1); // use DigIn1 (pin 6)
//   success = success && set_parameter<int16_t>(0x2317, 0x05, -16384); // Set offset so 50% => 0 mm/s (since [0%; 100%] = [0; 32767])
  
//    // Set gain 16384*gain = 100 000 um/s, gain = 10000/1638
//   // constexpr uint16_t NUM = 10000, DIV=1638;
//   // constexpr uint32_t GAIN = ((uint32_t)NUM)<<16 | DIV;

//   constexpr uint16_t NUM = PWM_MAX_SPEED, DIV=16384;
//   constexpr uint32_t GAIN = ((uint32_t)NUM)<<16 | DIV;
//   success = success && set_parameter<uint32_t>(0x2317, 0x04, GAIN);
  
//   return success;
// }

// void Motor::pwm_velocity_control_init()
// {
//   set_control_mode(MC3001::AnalogVelocityControl);
//   set_parameter<uint8_t>(0x2331,0x03, 7); // Set Target Velocity Source to PWM

//   begin_operation();
// }

// void Motor::fc_set_target_pos(float abs_pos){
//     set_parameter<int32_t>(0x607A,0x00, (int32_t)(abs_pos * 1000) ); // Set target pos
// }

// void Motor::prepare_force_control(){
//   set_operation_mode(0b1,0b1); // Zero set-point when changing mode (also set in init)
//   set_control_mode(MC3001::CSPosition);
// }

// DEBUGGING //
// void Motor::read_print_pwm_info(){
//   uint32_t freq;
//   int16_t duty;
//   int32_t value;
//   get_parameter<uint32_t>(0x2317, 0x02, freq);
//   get_parameter<int16_t>(0x2317, 0x03, duty);
//   get_parameter<int32_t>(0x2317, 0x06, value);
//   Serial << "Freq: " << freq << ", duty: " << duty << ", value: " << value << "\n";
// }


} // motor
