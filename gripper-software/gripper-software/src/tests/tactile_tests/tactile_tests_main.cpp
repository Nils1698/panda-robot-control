#include <Arduino.h>
#include "Streaming.h"
#include "UserStream.h"
#include "Listener.h"
#include "UserInterface/UserCommands/tactile_actions.h"

#include "fingers.h"

exec::Executer executer;
UserInput::Listener user_input;

std::vector<UserInput::UserAction> test_actions = {

};

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  user_serial::begin();
  debug_stream << "SETUP - TACTILE TEST\n";

  hw::Fingers::init();
}


UserInput::ActionList tactileactions = UserInput::tactile_actions("");

bool handle_userinput(const String& input_str){

  for(UserInput::UserAction& act : tactileactions.actions){
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