
#pragma once
#include "Motor.h"
#include "executer.h"
#include "UserAction.h"

#include <map>

namespace UserInput {

class motor_actions : public ActionList
{
public:
  motor_actions(String sub, exec::Executer &executor, motor_interface::Motor &motor) : ActionList(sub) {
    constexpr float max_width = 150; // TODO
    actions = {
      {"status", {
      }, [&](UserInput::arg_list_t args){
        motor.print_status(info_stream);
      }},

      {"s", {
      }, [&](UserInput::arg_list_t args){
        motor.shut_down();
      }},

      {"e", {
      }, [&](UserInput::arg_list_t args){
        motor.enable_operation();
      }},

      {"d", {
      }, [&](UserInput::arg_list_t args){
        motor.disable_operation();
      }},

      {"volt off", {
      }, [&](UserInput::arg_list_t args){
        motor.disable_voltage();
      }},

      {"sync", {
      }, [&](UserInput::arg_list_t args){
        motor.sync_state();
      }},

      {"set-speed", {
        {"speed mm/s", Parser::parse_arg<float>()}
      }, [&](UserInput::arg_list_t args){
        motor.set_goto_speed(args[0].get_float());
      }},

      // ACTIONS //
      {"halt", {
      }, [&](UserInput::arg_list_t args){
        motor.halt();
      }},

      {"unhalt", {
      }, [&](UserInput::arg_list_t args){
        motor.unhalt();
      }},

      {"q", {
      }, [&](UserInput::arg_list_t args){
        motor.quick_stop();
      }},

      {"limit", {
        {"width mm", Parser::parse_arg<float>({0,max_width})}
      }, [&](UserInput::arg_list_t args){
        motor.limit_grip(args[0].get_float());
      }},

      {"unlimit", {
      }, [&](UserInput::arg_list_t args){
        motor.unlimit_grip();
      }},


      {"home", {
      }, [&](UserInput::arg_list_t args){
        executor.execute([&]{
          motor.perform_homing();
        });      
      }},

      {"open", {
      }, [&](UserInput::arg_list_t args){
        executor.execute([&]{
          motor.open();
        });      
      }},

      {"goto", {
        {"width mm", Parser::parse_arg<float>()}
      }, [&](UserInput::arg_list_t args){
        auto w = args[0].get_float();
        executor.execute([&motor,w]{
          // motor.go_to(w,false,true);
          motor.go_to(w,false,false);
        });
      }},

      {"gorel", {
        {"change mm", Parser::parse_arg<float>()}
      }, [&](UserInput::arg_list_t args){
        auto delta = args[0].get_float();
        executor.execute([&motor,delta]{
          // motor.go_to(delta,true,true);
          motor.go_to(delta,true,false);
        });
      }},

      {"cmd-vel", {
        {"velocity mm/s",   Parser::parse_arg<float>({-200,200})},
        // {"stop force", Parser::parse_arg<float>({0,20 -1})}
      }, [&](UserInput::arg_list_t args){

        auto velocity   = args[0].get_float();
        // auto stop_force = args[1].get_float();
        motor.velocity_control_init(velocity);
        // executor.execute([&executor,&motor,stop_force]{
        //   hw::measure_tactile();
        //   if(stop_force>0 && hw::Fingers::total_force() > stop_force){
        //     // motor->velocity_control_set(0);
        //     executor.request_stop();
        //   }
        //   threads.yield();
        // },100,
        // [&motor]{
        //   motor.velocity_control_set(0);
        // });

      }},

     // PRINTING //
      {"pos", {
      }, [&](UserInput::arg_list_t args){
        executor.execute([&]{
          info_stream << motor.read_position() << "\n";
        });      
      }},

      // OTHER //
      {"disconnect", {
      }, [&](UserInput::arg_list_t args){
        mot_serial.end();
      }},

    };
  }
};

} //UserInput