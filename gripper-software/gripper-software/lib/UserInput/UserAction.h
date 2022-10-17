#pragma once

//#include "Listener.h"
#include <vector>
#include <functional>
#include "variant.h"

#include "Parser.h"

namespace UserInput {

struct any_arg {
    // https://stackoverflow.com/questions/72025874/proper-use-of-stdreference-wrapper-in-union
    String name;
    enum {
        INT='i',
        FLOAT='f',
        STRING='s'
    } type;
    union {
        Parser::parse_arg<int> 	 int_arg;
        Parser::parse_arg<float> float_arg;
    };
    any_arg(String name, Parser::parse_arg<int> pa)   :	name(name),type(INT)  {int_arg=pa;}
    any_arg(String name, Parser::parse_arg<float> pa) : name(name),type(FLOAT){float_arg=pa;}
    any_arg(String name)                              : name(name),type(STRING){}
}; 

typedef std::vector<UserInput::variant>& arg_list_t;

struct parse_result {
  std::vector<UserInput::variant> arguments;
  Parser parser;
};

struct UserAction {
  String name;
  std::vector<UserInput::any_arg> arg_types;
  std::function<void(arg_list_t)> fnc;

  parse_result try_parse(const String input){
    parse_result res = {{},{input}};
    auto &p = res.parser;

    if(p.match(name)){
      for(UserInput::any_arg &a : arg_types){
        switch(a.type)
        {
        case UserInput::any_arg::INT:{
          auto val = p.parse<int>(a.int_arg);
          res.arguments.push_back({val});
          break;}

        case UserInput::any_arg::FLOAT:{
          auto val = p.parse<float>(a.float_arg);
          res.arguments.push_back({val});
          break;}

        case UserInput::any_arg::STRING:{
          auto val = p.parse_string();
          res.arguments.push_back({val});
          break;}
        }
        
        if(p.parsing_error() != Parser::NO_ERROR) break;
      }      
      p.verify_input();
    }

    return res;
  }

  bool match_and_call(const String input){
    Parser p(input);
    if(p.match(name)){
      std::vector<UserInput::variant> args;
      for(UserInput::any_arg &a : arg_types){
        switch(a.type)
        {
        case UserInput::any_arg::INT:{
          auto val = p.parse<int>(a.int_arg);
          args.push_back({val});
          break;}

        case UserInput::any_arg::FLOAT:{
          auto val = p.parse<float>(a.float_arg);
          args.push_back({val});
          break;}

        case UserInput::any_arg::STRING:{
          auto val = p.parse_string();
          args.push_back({val});
          break;}

        }
        
      }
      if(p.verify_input()){
        fnc(args);
        return true;
      }

    }
    return false;
  }
};

class ActionList {
public:
  const String category;
  std::vector<UserAction> actions; // TODO make private
public:
  ActionList(String category) : category(category){}
  ActionList(std::vector<UserAction> &actions) : category(""), actions(actions){}
};

} // UserInput