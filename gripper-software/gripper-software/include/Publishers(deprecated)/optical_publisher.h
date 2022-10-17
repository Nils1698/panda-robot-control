#pragma once

#include "Arduino.h"
#include "publishable.h"
#include "fingers.h"

typedef uint8_t specifier_t;
enum optical_specifier_t : specifier_t {
    RAW = 0
};

struct optical_publisher : public Publishable
{
    const static int TYPE_ID = __COUNTER__;
    const specifier_t spec;
    hw::Fingers::fingers_t *fingers;
    optical_publisher(optical_specifier_t spec=RAW, hw::Fingers::fingers_t *fingers=nullptr) : Publishable(TYPE_ID, spec), spec(spec), fingers(fingers) {}

    void publish(Print &out) override {
        // TODO acquire sensor

        if(spec == RAW){
            auto &s1 = (*fingers)[0]->opticalSensor;
            auto &s2 = (*fingers)[1]->opticalSensor;
            out << _FLOAT(s1.slideX(),3) << " " << _FLOAT(s1.slideY(),3) << " ";
            out << _FLOAT(s2.slideX(),3) << " " << _FLOAT(s2.slideY(),3) << " ";
        }
        // TODO release sensor
    }
    void printTo(Print &out) override {
        switch(spec)
        {
        case RAW:
            out << "raw";
            break;
        }
    }
};