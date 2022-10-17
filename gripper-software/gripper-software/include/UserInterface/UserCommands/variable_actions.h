#pragma once

#include <map>
#include "variant.h"

namespace UserInput {

inline bool variable_actions(const String input, std::map<String, variant> &user_vars){

    Parser parser(input);

    if(parser.match("set")){
      for(auto &x : user_vars){
          const auto& var_name = x.first;
          variant& var = x.second;
          if(parser.match(var_name)){
            switch(var.type())
            {
            case variant::INT:{
              int i = parser.parse<int>();
              if(parser.verify_input()){
                var.set_int(i);
                return true;
              }
              break;}

            case variant::FLOAT:{
              auto f = parser.parse<float>();
              if(parser.verify_input()){
                var.set_float(f);
                return true;
              }
              break;}            

            case variant::STRING:{
              auto str = parser.parse_string();
              if(parser.verify_input()){
                var.set_string(str);
                return true;
              }
              break;}
            }            

          }
      }
    }

    else if(parser.match("get")){
      for (auto &x : user_vars){
          if(parser.match(x.first)){
            variant &var = x.second;
            switch(var.type())
            {
            case variant::INT:{
              if(parser.verify_input()){
                info_stream << x.first << "=" << var.get_int() << "\n";
                return true;
              }
              break;}

            case variant::FLOAT:{
              if(parser.verify_input()){
                info_stream << x.first << "=" << var.get_float() << "\n";
                return true;
              }
              break;}

            case variant::STRING:{
              if(parser.verify_input()){
                info_stream << x.first << "=" << '"' << var.get_string() << "\"\n";
                return true;
              }
              break;}

            }            
            
          }
      }
    }  

    return false;
}

}