#include "Button.h"

#include "UserStream.h"

namespace hw {

void Button::init(){
    pinMode(PIN, INPUT_PULLUP);
}

void Button::update_state(){
    bool readState = digitalRead(PIN)==LOW;
    if(readState != newButtonState){
        debounceInit = millis();
        newButtonState = readState;
    }
    if(millis()-debounceInit > DEBOUNCE_DELAY){
        buttonState = newButtonState;
    }
}

bool Button::is_pressed(){
    return buttonState;
}

Button::event_t Button::read_events(){
    update_state();
    if(millis() - last_button_event.time2 < 100) return NONE;

    switch (last_button_event.type)
    {
    case NONE:
        if(is_pressed()){
            last_button_event.type = PRESS;
            last_button_event.time = millis();
            return PRESS;
        }
        break;

    case PRESS:
        if(!is_pressed()){
            last_button_event.type = RELEASE;
            last_button_event.time2 = millis();
            return RELEASE;
        }else if(millis() - last_button_event.time >= HOLD_TIME){
            last_button_event.type = HOLD;
            last_button_event.time2 = millis();
            return HOLD;
        }
        break;

    case RELEASE:
        if(millis() - last_button_event.time >= DOUBLE_CLICK_TIME){
            last_button_event.type = NONE;
            last_button_event.time2 = millis();
            return CLICK;
        }else if(is_pressed()){
            last_button_event.type = DOUBLE_CLICK;
            last_button_event.time2 = millis();
            return DOUBLE_CLICK;
        }
        break;

    case DOUBLE_CLICK:
        if(!is_pressed()){
            last_button_event.type = NONE;
            last_button_event.time2 = millis();
        }
        break;

    case HOLD:
        if(!is_pressed()){
            last_button_event.type = NONE;
            last_button_event.time2 = millis();
            return LONG_CLICK;
        }
        break;

    default:
        break;
    }
    
    return NONE;
}
} //hw