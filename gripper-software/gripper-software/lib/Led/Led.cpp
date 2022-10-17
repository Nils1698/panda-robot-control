#include "Led.h"

#include "executer.h"

exec::Executer blinker;

namespace hw {

void Led::init(){
    pinMode(PIN, OUTPUT);
    off();
}

void Led::on(){
    blinker.request_stop();
    digitalWrite(PIN, HIGH);
    _is_on = true;
}

void Led::off(){
    blinker.request_stop();
    digitalWrite(PIN, LOW);
    _is_on = false;
}

void Led::set_state(bool new_state){
    _is_on = new_state;
    digitalWrite(PIN, new_state ? HIGH : LOW);
}

void Led::blink(unsigned long delay_ms){
    if(blinker.is_executing()){blinker.kill_execution();}
    blinker.execute_loop([&]{
        set_state(!_is_on);
    }, delay_ms);
}


} //hw