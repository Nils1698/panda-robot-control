#include <Arduino.h>
#include "Streaming.h"
#include "TeensyThreads.h"

#include "executer.h"
#include "UserInterface/UserCommands/motor_actions.h"
#include "UserStream.h"

#include "Listener.h"
#include "Motor.h"
#include <map>

exec::Executer executer;
UserInput::Listener user_input;

namespace hw {
  motor_interface::Motor _motor;
  inline motor_interface::Motor *motor(){
    return &_motor;
  }
  inline motor_interface::Motor &motor_ref(){
    return _motor;
  }
}

std::map<String, MC3001::save_configuration_options> save_conf_map {
  {"conf",  MC3001::SAVE_APP_PARAMS},
  {"m3216", MC3001::SAVE_TO_APP1}, // currently used motor
  {"m2250", MC3001::SAVE_TO_APP2}, // previously used motor
};
std::map<String, MC3001::load_configuration_options> load_conf_map {
  {"conf",  MC3001::LOAD_APP_PARAMS},
  {"m3216", MC3001::LOAD_FROM_APP1}, // currently used motor
  {"m2250", MC3001::LOAD_FROM_APP2}, // previously used motor
};


std::vector<UserInput::UserAction> test_actions = {
  {"test", {
    {"int arg",   UserInput::Parser::parse_arg<int>()},
    {"str arg"},
    {"float arg", UserInput::Parser::parse_arg<float>()}
  }, [](UserInput::arg_list_t args){
    debug_stream << args[0].get_int();
    debug_stream << args[1].get_string();
    debug_stream << args[2].get_float();
    debug_stream << "test\n";
  }},

  // {"home-in", {
  // }, [](UserInput::arg_list_t args){
  //   hw::motor()->perform_homing_in();
  // }},
  // {"home-out", {
  // }, [](UserInput::arg_list_t args){
  //   hw::motor()->perform_homing_out();
  // }},

  {"stop", {
  }, [](UserInput::arg_list_t args){
    executer.request_stop();
  }},

  {"get-param", {
    {"type"},
    {"index",     UserInput::Parser::parse_arg<int>()},
    {"sub index", UserInput::Parser::parse_arg<int>()}
  }, [](UserInput::arg_list_t args){
    String type   = args[0].get_string();
    int index     = args[1].get_int();
    int sub_index = args[2].get_int();
    info_stream << "0x" << _HEX(index) << "." << _HEX(sub_index) << ": ";
    if(type=="u32"){
      uint32_t val=0;
      hw::motor()->get_parameter(index,sub_index, val);
      info_stream << val << "\n";
    }
    if(type=="u16"){
      uint16_t val=0;
      hw::motor()->get_parameter(index,sub_index, val);
      info_stream << val << "\n";
    }
  }},

  {"step-vel", {
    {"vel 1",     UserInput::Parser::parse_arg<float>()},
    {"vel 2",     UserInput::Parser::parse_arg<float>()},
  }, [](UserInput::arg_list_t args){
    float vel1 = args[0].get_float();
    float vel2 = args[1].get_float();

    hw::motor()->velocity_control_init(vel1);

    time_t timeStart = millis();
    time_t last_sample = 0;
    time_t time_passed=0;
    bool wait_step = true;
    while(time_passed < 2000)
    {
      time_passed = millis() - timeStart;
      if(wait_step && time_passed > 1000){
        hw::motor()->velocity_control_set(vel2);
        wait_step=false;
      }
      if(millis()-last_sample >= 5){
        last_sample=millis();
        signal_stream << time_passed << " " << hw::motor()->read_velocity() << "\n";
      }
    }
    hw::motor()->velocity_control_set(0);        

  }},

  {"tect-comm-speed", {
    {"messages",  UserInput::Parser::parse_arg<int>()},
  }, [](UserInput::arg_list_t args){
    int n_msg = args[0].get_int();
    executer.execute([=](){
      info_stream << "Testing communication speed on " << n_msg << " messages..\n";
      info_stream << "Write [us], Read [us]\n";
      for (int i = 0; i < n_msg; i++){
          info_stream << hw::motor()->test_write_delay() << ", "
                      << hw::motor()->test_read_delay()  << "\n";
          threads.delay(10);
      }
    });

  }},

  {"configure-feed", {
  }, [](UserInput::arg_list_t args){

    if(hw::motor()->reset_gripperFeed()){
      int32_t pos_in, pos_out;
      hw::motor()->shut_down();
      info_stream << "Close gripper fingers and press enter ..\n";
      user_input.await_user_string();
      pos_in = hw::motor()->read_raw_position();
      info_stream << pos_in << "\n";
      info_stream << "Open gripper fingers and press enter ..\n";
      user_input.await_user_string();
      pos_out = hw::motor()->read_raw_position();
      info_stream << pos_out << "\n";

      info_stream << "Measured stroke (mm): \n";
      float stroke;
      stroke = user_input.await_user_float({0,500});
      info_stream << "Stroke: " << stroke << "\n";
      const float feed = stroke*1e6 / (pos_out-pos_in);
      info_stream << "Feed: " << feed << " um/rev\n";

      info_stream << "Use new feed? (y/n)\n";
      if(user_input.await_user_string() == "y"){
        hw::motor()->set_gripperFeed(feed);
        info_stream << "Perform homing? (y/n)\n";
        if(user_input.await_user_string() == "y"){
          hw::motor()->enable_operation();
          hw::motor()->perform_homing();
          const float conf_pos = hw::motor()->read_position();
          info_stream << "Current pos: "<< conf_pos << "mm\n";
        }

        info_stream << "Save in configuration? (y/n)\n";
        if(user_input.await_user_string() == "y"){
          debug_com_stream.setActive(true);
          if(hw::motor()->save_configuration(MC3001::SAVE_APP_PARAMS)){
            info_stream << "Saved\n";
          }else{
            info_stream << "Saving failed\n";
          }
          debug_com_stream.setActive(false);
        }

      }
      info_stream << "Configuration done!\n";
    }else{
        info_stream << "Test failed\n";
    }        

  }},

  {"save-conf", {
    {"conf"},
  }, [](UserInput::arg_list_t args){
    for (auto const& conf : save_conf_map){
        if(args[0].get_string()==conf.first){
          debug_com_stream.setActive(true);
          debug_stream <<"saving ... ";
          if(hw::motor()->save_configuration(conf.second)){
            info_stream << "Saved\n";
          }else{
            info_stream << "Saving failed\n";
          }      
          debug_com_stream.setActive(false);            
        }
    }
  }},

  {"load-conf", {
    {"conf"},
  }, [](UserInput::arg_list_t args){
    for (auto const& conf : load_conf_map){
        if(args[0].get_string()==conf.first){
          debug_com_stream.setActive(true);
          debug_stream <<"loading ... ";
          if(hw::motor()->load_configuration(conf.second)){
            info_stream << "Loaded\n";
          }else{
            info_stream << "Loading failed\n";
          }      
          debug_com_stream.setActive(false);            
        }
    }
  }},

  {"reset-all", {
  }, [](UserInput::arg_list_t args){
    debug_stream <<"reset-all\n";
    debug_com_stream.setActive(true);
    if(hw::motor()->load_configuration(MC3001::RESET_ALL_PARAMS)){
      info_stream << "Reset!\n";
    }else{
      info_stream << "Resetting failed\n";
    }      
    debug_com_stream.setActive(false);
  }},


};

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  user_serial::begin();
  debug_stream << "SETUP - MOTOR TESTS\n";

  hw::motor()->init();

  delay(500);
  debug_com_stream.setActive(false);
  debug_stream.setActive(true);
}

UserInput::ActionList motoractions = UserInput::motor_actions("", executer, hw::motor_ref());

bool handle_userinput(const String& input_str){

  for(UserInput::UserAction& act : motoractions.actions){
    if(act.match_and_call( input_str )){ 
      return true;
    }
  }
  for(UserInput::UserAction& act : test_actions){
    if(act.match_and_call( input_str )){ 
      return true;
    }
  }
  return false;
}

void loop()
{

  user_input.update_reception();
  if(user_input.has_input()){
    if( !handle_userinput(user_input.get_input()) )
    {
      info_stream << "unknown command\n";
    }
  }

  threads.yield();
}