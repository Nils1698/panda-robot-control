#pragma once 

#include "Arduino.h"
#include "publishable.h"

#include "Motor.h"

enum motor_specifier_t : uint8_t {
    POSITION,
    VELOCITY,
    TORQUE
};
struct motor_publisher : public Publishable
{
    const static int TYPE_ID = __COUNTER__;

    const motor_specifier_t spec;
    motor_interface::Motor * const motor;
    motor_publisher(motor_specifier_t spec, motor_interface::Motor * const motor=nullptr) : Publishable(TYPE_ID, spec), spec(spec), motor(motor) {}

    void publish(Print &out) override {
        // TODO acquire motor
        switch(spec)
        {
        case POSITION:
            out << motor->get_position() << " ";
            break;
        case VELOCITY:
            out << motor->get_velocity() << " ";
            break;
        case TORQUE:
            out << motor->get_torque_raw() << " ";
            break;            
        }
        // TODO release motor
    }
    void printTo(Print &out){
        switch(spec)
        {
        case POSITION:
            out << "Pos";
            break;
        case VELOCITY:
            out << "Vel";
            break;
        case TORQUE:
            out << "Torque";
            break;            
        }

    }

};