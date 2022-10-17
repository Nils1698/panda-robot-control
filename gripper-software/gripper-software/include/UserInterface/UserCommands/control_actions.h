
#pragma once
#include "executer.h"
#include "ForceController.h"
#include "UserAction.h"

namespace UserInput {

class ctrl_actions : public ActionList
{
public:
  ctrl_actions(String sub, exec::Executer &executor, ForceController* &controller) : ActionList(sub) {

    actions = {
      {"start", {
        {"force", Parser::parse_arg<float>()}
      }, [&](UserInput::arg_list_t args){
        float force = args[0].get_float();
        executor.execute([&controller, &executor, force]{
          controller->run(force, [&executor](){return executor.is_ok();});
        });        
      }},

      {"set", {
        {"force", Parser::parse_arg<float>()}
      }, [&](UserInput::arg_list_t args){
        float force = args[0].get_float();
        controller->set_force(force);
      }},

      {"stop", {
      }, [&](UserInput::arg_list_t args){
        controller->stop();
      }},

      {"print", {
      }, [&](UserInput::arg_list_t args){
        controller->print_params(info_stream);
      }},

    };
  }
};

} //UserInput