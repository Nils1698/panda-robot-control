#include <Arduino.h>
#include "Streaming.h"
#include "TeensyThreads.h"

#include "UserStream.h"

#include "executer.h"

#include "HW_prototype.h"

#include "Listener.h"
#include "UserInterface/UserCommands/motor_actions.h"
#include "UserInterface/UserCommands/control_actions.h"
#include "UserInterface/UserCommands/tactile_actions.h"
#include "UserInterface/UserCommands/optical_actions.h"
#include "UserInterface/UserCommands/variable_actions.h"
#include "UserInterface/Publishing/publishers.h"

#include "demo_actions.h"

#include "tactile_calibration.h"
#include "PIDaI_controller.h"
#include "detection_controller.h"

#include "gripper_actions.h"

#include <memory>
#include <numeric> // accumulate
#include <vector>

#include <map>

#include "UserInterface/Publishing/pubsub.h"
#include "Parser.h"

UserInput::Listener user_input;

exec::Executer publisher("Publisher"), slipDetector("Detector");

PUB::pubsub ps({
    PUB::tactile_publisher("S1", hw::Fingers::F[0]->TactileSensor),
    PUB::tactile_publisher("S2", hw::Fingers::F[1]->TactileSensor),
    PUB::motor_publisher(hw::motor_ref()),
    PUB::control_publisher(GripperActions::controller),
    PUB::perception_publisher(hw::Fingers::F),
});

// NOTE: user_vars should only be accesed from the main thread
std::map<String, UserInput::variant> user_vars = {
  { "pub-period",      UserInput::variant(100)},
  { "meas-period",     UserInput::variant(100)},
  { "detect-force",  UserInput::variant(0.1f)},
  { "btn_hold_cmd",  UserInput::variant("help")},
  { "btn_dbclk_cmd", UserInput::variant("open")},
};
const int   &pub_period         = user_vars["pub-period"].int_ref();
const int   &meas_period        = user_vars["meas-period"].int_ref();
const float &detection_force  = user_vars["detect-force"].float_ref();
String &btn_hold_cmd          = user_vars["btn_hold_cmd"].string_ref();
String &btn_dbclick_cmd       = user_vars["btn_dbclk_cmd"].string_ref();


void grip(const float force = 0.5){
  GripperActions::executer.execute([force]()
  {
    TactileCalibration::zcal();
    GripperActions::tactile_detect(max( detection_force, force/2-2), 50);
    debug_stream << "Detected! Start control..\n";
    debug_stream << "Start Control\n";
    GripperActions::hold_force(force);
  });
}

void stream_contact_info(){
  GripperActions::executer.execute_loop([]()
  {
    float obj_width = hw::motor()->read_position();
    hw::Fingers::measure_tactile();
    
    TactilePerception::contact_info_t ci0 = TactilePerception::compute_contact_info( hw::Fingers::F[0]->TactileSensor.forces(), 0.1, 0.7);
    TactilePerception::contact_info_t ci1 = TactilePerception::compute_contact_info( hw::Fingers::F[1]->TactileSensor.forces(), 0.1, 0.7);
    debug_stream << "Width" << obj_width << "\n";
    debug_stream << "Area:\t" << ci0.area << "\t/\t" << ci1.area << "\n";
    debug_stream << "Cx:\t" << ci0.cm_x << "\t/\t" << ci1.cm_x << "\n";
    debug_stream << "Cy:\t" << ci0.cm_y << "\t/\t" << ci1.cm_y << "\n";

    float tau_x = -0.5*(ci0.cm_y - ci1.cm_y) * (ci0.total_force - ci1.total_force);
    debug_stream << "tau_x:\t" << tau_x << "\n";

    //debug_stream << "Smooth:\t" << smoothness(hw::Fingers::F[0]->TactileSensor.forces()) << "\t/\t" << smoothness(hw::Fingers::F[1]->TactileSensor.forces()) << "\n";

    // float sm0 = TactilePerception::smoothness2(hw::Fingers::F[0]->TactileSensor.forces());
    // float sm1 = TactilePerception::smoothness2(hw::Fingers::F[1]->TactileSensor.forces());
    // debug_stream << "Smooth2:\t" << sm0 << "\t/\t" << sm1 << "\n";

    // signal_stream << obj_width << " "
    //               << ci0.area+ci1.area << " "
    //               << sm0+sm1 << " "
    //                << "\n";    
  }, 100);
}

void stream_moment_info(){
  GripperActions::executer.execute_loop([]()
  {
    hw::Fingers::measure_tactile();
    TactilePerception::compute_moment_info();
  }, 200);  
}


void open(){
  if(slipDetector.is_executing()){
    slipDetector.request_stop();
  }
  if(GripperActions::controller->is_active()){
    GripperActions::controller->stop();
    threads.delay(100);
  }
  if(GripperActions::executer.is_executing()){
    GripperActions::executer.request_stop();
  }
  hw::motor()->open();
}

extern std::vector<UserInput::ActionList> action_lists;
std::vector<UserInput::UserAction> user_actions = {

  {"help", {}, [](UserInput::arg_list_t args){
    info_stream << "Variables :\n";
    for (auto const& v : user_vars){
      info_stream << "  " << v.second.type_char() << ": " << v.first << "\n";
    }
    info_stream << "Subscriptions:\n";

    ps.printOptions(info_stream);

    info_stream << "Commands:\n";
    info_stream << "  set [var name] [value]\n";
    info_stream << "  get [var name]\n";
    for(auto &al : action_lists){
      if(al.category != ""){
        info_stream << "(" << al.category << ")\n";
      }
      for(UserInput::UserAction& act : al.actions){
        info_stream << "  " << al.category << " " << act.name << " ";
        for(auto at : act.arg_types){
          info_stream << " ["<< (char)at.type<< ": " << at.name << "] ";
        }
        info_stream << "\n";
        }
    }
  }},

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

  {"open", {
  }, [](UserInput::arg_list_t args){
    open();
  }},

  {"state", {
  }, [](UserInput::arg_list_t args){
    GripperActions::executer.printTo(info_stream);
    publisher.printTo(info_stream);
    slipDetector.printTo(info_stream);
    info_stream << "controller " << (GripperActions::controller->is_active() ? "is running" : "not running") << "\n";
  }},

  {"sub", {
    {"type"},
    {"category"},
    {"specifier",   UserInput::Parser::parse_arg<int>(0)}
  }, [](UserInput::arg_list_t args){
    ps.subscribe(args[0].get_string(),args[1].get_string(),args[2].get_int());
    info_stream << "Subscriptions:\n";
    ps.printSubscription(info_stream);
  }},

  {"sub", {
    {"type"},
    {"category"},
    {"specifier"}
  }, [](UserInput::arg_list_t args){
    ps.subscribe(args[0].get_string(),args[1].get_string(),args[2].get_string());
    info_stream << "Subscriptions:\n";
    ps.printSubscription(info_stream);
  }},

  {"unsub", {
  }, [](UserInput::arg_list_t args){
    ps.unsubscribe_all();
    info_stream << "Subscriptions cleared\n";
  }},

  {"pub-test", {
  }, [](UserInput::arg_list_t args){
    ps.publish_subscriptions(info_stream);
  }},

  {"debug", {
    {"on/off"}
  }, [](UserInput::arg_list_t args){
    if(args[0].get_string()=="on")       debug_stream.setActive(true);
    else if(args[0].get_string()=="off") debug_stream.setActive(false);
  }},

  {"warnings", {
    {"on/off"}
  }, [](UserInput::arg_list_t args){
    if(args[0].get_string()=="on")       warning_stream.setActive(true);
    else if(args[0].get_string()=="off") warning_stream.setActive(false);
  }},

  {"stop", {
  }, [](UserInput::arg_list_t args){
    GripperActions::executer.request_stop();
    GripperActions::controller->stop();
  }},

  {"pub-on", {
  }, [](UserInput::arg_list_t args){
    publisher.execute_loop([]{
      ps.publish_subscriptions(signal_stream);
    }, pub_period);    
  }},

  {"pub-off", {
  }, [](UserInput::arg_list_t args){
    publisher.request_stop();    
  }},

  {"close", {
    {"force", UserInput::Parser::parse_arg<float>()}
  }, [](UserInput::arg_list_t args){
    auto force = args[0].get_float();
    GripperActions::executer.execute([force]{
      GripperActions::tactile_close(force);
    });    
  }},

  {"grip", {
    {"force", UserInput::Parser::parse_arg<float>()}
  }, [](UserInput::arg_list_t args){
    grip(args[0].get_float());
  }},

  {"hold", {
    {"force", UserInput::Parser::parse_arg<float>({0,20})}
  }, [](UserInput::arg_list_t args){
    auto force = args[0].get_float();
    GripperActions::executer.execute([force]{
      GripperActions::hold_force(force);
    });
  }},

  {"detect", {
    {"speed", UserInput::Parser::parse_arg<float>({0,150, 50})}
  }, [](UserInput::arg_list_t args)
  {
    auto spd = args[0].get_float();
    GripperActions::executer.execute([spd]{
      TactileCalibration::zcal();
      auto res = GripperActions::tactile_detect(detection_force, spd);
      debug_stream << "result: " << res << "\n";
    });    
  }},

  {"release", {
  }, [](UserInput::arg_list_t args){
    GripperActions::executer.execute([]{
      GripperActions::tactile_release();
    });    
  }},

  {"squeeze", {
  }, [](UserInput::arg_list_t args){
    GripperActions::executer.execute([]{
      GripperActions::tactile_squeeze();
    });    
  }},


  {"empty", {
  }, [](UserInput::arg_list_t args){
    
  }},

};

std::vector<UserInput::ActionList> action_lists = {
  UserInput::ActionList(user_actions),
  UserInput::ctrl_actions("ctrl", GripperActions::executer, GripperActions::controller),
  UserInput::motor_actions("m", GripperActions::executer, hw::motor_ref()),
  UserInput::tactile_actions("tact"),
  UserInput::optical_actions("opt", hw::Fingers::F),
  UserInput::demo_actions()
};

void setup()
{
  hw::led()->init();
  hw::led()->blink(600);
  user_serial::begin();

  // Wait for "init"
  while(true){
    user_input.update_reception();
    if(user_input.has_input() && user_input.get_input()=="init"){
      break;
    }
    threads.yield();
  }

  hw::circII.debug_non_func(); // TODO if removing this line, teensy stops working for some reason ..??? Something inside Circiut_2x7.cpp needs to be called somewhere

  debug_com_stream.setActive(false);
  debug_stream.setActive(true);
  debug_stream << "ROBOTBRAG SETUP\n";

  hw::init();
  delay(500);  

  TactileCalibration::cal_load();

  // const float thres = 0.015;
  const float thres = 0.05;
  for (size_t i = 0; i < hw::Fingers::F[0]->TactileSensor.n_taxels(); i++) hw::Fingers::F[0]->TactileSensor.set_threshold(i, thres);
  for (size_t i = 0; i < hw::Fingers::F[1]->TactileSensor.n_taxels(); i++) hw::Fingers::F[1]->TactileSensor.set_threshold(i, thres);

  // pid_controller.set_params(2,0,500,-0.95);
  GripperActions::pid_controller.set_params(0.5,0,100,-0.95); // Tuned slower since calibration gives artificially high forces
  GripperActions::detect_controller.set_params(15,0,1000,-0.95); // TODO tune to high response on low forces
}

void calib_thres(){
  GripperActions::executer.execute([]{
    for (size_t i = 0; i < hw::Fingers::N_TAXELS; i++) hw::Fingers::F[0]->TactileSensor.set_threshold(i, -1);
    for (size_t i = 0; i < hw::Fingers::N_TAXELS; i++) hw::Fingers::F[1]->TactileSensor.set_threshold(i, -1);
    TactileCalibration::zcal();

    int n=0;
    float mu[2*hw::Fingers::N_TAXELS], mu2[2*hw::Fingers::N_TAXELS], amax[2*hw::Fingers::N_TAXELS]={0};

    // auto &s = hw::Fingers::F[0]->TactileSensor;
    unsigned int loop_period = 20;
    unsigned long last_fnc_call=0;
    while(GripperActions::executer.is_ok()){
        if(millis() - last_fnc_call >= loop_period){
            last_fnc_call = millis();
            

            hw::measure_tactile();
            n++;
            for(size_t si = 0; si < 2; si++){
              auto &s = hw::Fingers::F[si]->TactileSensor;
              for(size_t i = 0; i < hw::Fingers::N_TAXELS; i++){
                mu[i]  += (s.force(i) - mu[i]) / n;              // update average
                mu2[i] += (s.force(i)*s.force(i) - mu2[i]) / n; // update sqaured average
                if( fabs(s.force(i)) > amax[i] ) amax[i]=fabs(s.force(i)); // update max
                debug_stream << _FLOAT(mu[i],4) << "(" << s.force(i) << ") ";
                // debug_stream << _FLOAT(var[i],4) << "(" << s.force(i) << ") ";
              }
            }
            debug_stream << "\n";

        }
        threads.yield();
    }


    for(size_t si = 0; si < 2; si++){
      auto &s = hw::Fingers::F[si]->TactileSensor;

      for (float m : mu){debug_stream << _FLOAT(m,4) << " ";}
      debug_stream << "\n";
      for (size_t i = 0; i < hw::Fingers::N_TAXELS; i++){
        float var = fabs(mu2[i]-mu[i]*mu[i]);
        debug_stream << _FLOAT(sqrt(var),4) << " ";
        s.set_cusum_w(i,3*sqrt(var));
      }
      debug_stream << "\n";
      for (size_t i = 0; i < hw::Fingers::N_TAXELS; i++){
        debug_stream << _FLOAT(amax[i],4) << " ";
        s.set_threshold(i, amax[i]*1.5);
      }
      debug_stream << "\n";
    }

  });
}

bool handle_userinput(const String& input_str)
{
  if(UserInput::variable_actions(input_str, user_vars)) return true;

  UserInput::Parser parser(input_str);

  for(auto &al : action_lists){
    if(al.category=="" || parser.match(al.category)){
      
      for(UserInput::UserAction& act : al.actions){

        if(act.match_and_call( parser.rest_string() )){ 
          return true;
        }
      }

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
      info_stream << "unknown command: " << user_input.get_input() << "\n";
    }
  }

  hw::Button::event_t event = hw::button()->read_events();

  if(event == hw::Button::CLICK){
    debug_stream << "Button click\n";
  }else if(event == hw::Button::DOUBLE_CLICK){
    debug_stream << "Button double click\n";
    user_input.set_input(btn_dbclick_cmd);
  }else if(event == hw::Button::HOLD){
    debug_stream << "Button hold\n";
    user_input.set_input(btn_hold_cmd);
  }

  // DEFAULT EXECUTION
  if(!GripperActions::executer.is_executing()){
    debug_stream << "Start default execution\n";
    GripperActions::executer.execute_receding([]
    { // Will be overtaken by any other execution call

      hw::Fingers::measure_tactile();
      hw::motor()->read_position();
      hw::motor()->read_velocity();
      hw::motor()->read_torque_raw();
    }, meas_period);
  }

  threads.yield();
}