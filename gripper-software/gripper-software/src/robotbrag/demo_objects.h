#pragma once

#include "Arduino.h"
#include "Streaming.h"

struct DemoParam {
    float exp,min,max;
    DemoParam() : exp(0), min(0), max(0){}
    DemoParam(float exp, float pm) : exp(exp), min(exp-pm), max(exp+pm){}
    DemoParam(float exp, float lower, float upper) : exp(exp), min(lower), max(upper){}
    bool matches(float val){
        return val>=min && val<=max;
    }
    float residual(float val){
        return fabs( (val-exp)/exp );
    }
    void printTo(Print &out){
        out << exp << "[" << min << "," << max << "]";
    }
};
struct DemoObject {
    const String name;
    DemoParam width, hardness, area, flatness;
    float grip_force;
    bool matches(float obj_width, float obj_hardness, float obj_area, float obj_flatness){
        return width.matches(obj_width) && hardness.matches(obj_hardness) && area.matches(obj_area) /*&& obj.flatness.matches(obj_flatness)*/;
    }
    void printTo(Print &out){
        out << name << "(";
        width.printTo(out);
        out << "mm, ";
        hardness.printTo(out);
        out << "N/m, ";
        area.printTo(out);
        out << "%, ";
        flatness.printTo(out);
        out << " : " << grip_force << "N)";
    }
};
