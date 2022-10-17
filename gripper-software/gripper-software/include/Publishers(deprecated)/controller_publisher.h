#pragma once

#include "Arduino.h"
#include "publishable.h"
#include <map>

#include "ForceController.h"

struct controller_publisher : public Publishable
{
    enum control_specifier_t : uint8_t {
        DES_FORCE,
        FORCE_ERROR,
        CMD_VEL
    };
    static std::map<control_specifier_t, String> subscription_names(){return {
        {DES_FORCE, "des_force"},
        {FORCE_ERROR, "err"},
        {CMD_VEL, "cmd_vel"},
    };}

    const static int TYPE_ID = __COUNTER__;
    control_specifier_t spec;
    ForceController &controller;
    controller_publisher(control_specifier_t spec, ForceController &controller) : Publishable(TYPE_ID, spec), spec(spec), controller(controller) {}

    void publish(Print &out) override {
        // TODO acquire controller
        switch (spec)
        {
        case DES_FORCE:
            out << controller.get_desired_force() << " ";
            break;
        case FORCE_ERROR:
            out << controller.get_error()  << " ";
            break;
        case CMD_VEL:
            out << controller.get_cmd_vel() << " ";
            break;
        }
        //controller.print_state(out);
        // TODO release controller
    }
    void printTo(Print &out) override {
        out << subscription_names().at(spec) << " ";
    }
};
