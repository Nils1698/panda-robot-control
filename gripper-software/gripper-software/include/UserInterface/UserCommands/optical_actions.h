
#pragma once
#include "executer.h"
#include "fingers.h"

namespace UserInput {

class optical_actions : public ActionList
{
public:
  optical_actions(String sub, hw::Fingers::fingers_t &F) : ActionList(sub) {
    actions = {
      {"led-on", {
      }, [&](UserInput::arg_list_t args){
        F[0]->opticalSensor.setLed(true);
        F[1]->opticalSensor.setLed(true);        
      }},
      {"led-off", {
      }, [&](UserInput::arg_list_t args){
        F[0]->opticalSensor.setLed(false);
        F[1]->opticalSensor.setLed(false);        
      }},
    };
  }
};

}