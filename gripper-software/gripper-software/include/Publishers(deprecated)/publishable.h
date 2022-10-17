
#pragma once
#include "Arduino.h"

struct Publishable
{
    const int type_id;
    const uint8_t specifier;
    Publishable(const int type, uint8_t spec) : type_id(type), specifier(spec){}
    virtual void publish(Print &out)=0;
    virtual void printTo(Print &out)=0;
};