#pragma once

#include "Arduino.h"
#include "UserStream.h"

namespace UserInput {

struct variant {
    enum type_t {
      INT = 'i',
      FLOAT = 'f',
      STRING = 's'
    };
  private:
    const type_t _type;
    union {
        int i;
        float f;
        String* str;
    };
    // Threads::Mutex mutex;     - Not needed since only accessed by user-com thread

  public:
    variant()         : _type(variant::INT)  {i=0;}
    type_t type() const {return _type;}
    char type_char() const {return _type;}

    // INT
    variant(int v)    : _type(variant::INT)  {i=v;}
    int& int_ref(){
      if(_type!=variant::INT){type_error(INT);}
      return i;
    }
    void set_int(int v){int_ref() = v;}
    int get_int(){return int_ref();}

    // FLOAT
    variant(float v)  : _type(variant::FLOAT){f=v;}
    float& float_ref(){
      if(_type!=variant::FLOAT){type_error(FLOAT);}
      return f;
    }
    void set_float(float v){float_ref() = v;}
    float get_float() {return float_ref();}

    // STRING
    variant(String v)  : _type(variant::STRING){str=new String(v);}
    void set_string(String v){
      if(_type!=variant::STRING){type_error(STRING);}
      delete str;
      str = new String(v);
    }
    String& string_ref(){
      if(_type!=variant::STRING){type_error(STRING);}
      return *str;
    }
    String get_string(){
      if(_type!=variant::STRING){type_error(STRING);}
      return *str;
    }

    variant(const variant& v) : _type(v._type){
      switch (v._type) {
        case INT:   i=v.i; break;
        case FLOAT: f=v.f; break;
        case STRING: str=new String(*v.str); break;
        default:
          error_stream<<"variant copy error!\n";
          break;
      }
    }

    ~variant(){
      if(_type==STRING){delete str;}
    }

  private:
    void type_error(){
      error_stream << "Wrong type in variant!\n";
    }
    void type_error(type_t type){
      error_stream << "Wrong type in variant ("<< (int)type <<" != " << (int)_type << ")!\n";
    }
};

} // UserInput