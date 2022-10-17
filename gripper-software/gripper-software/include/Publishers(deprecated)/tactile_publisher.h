#pragma once

#include "Arduino.h"
#include "publishable.h"
#include "fingers.h"

#include <map>

// TODO inclose in namespace:
enum taxel_specifier_t : uint8_t {
    FORCE       = 0,
    RAW_SIG     = 1,
    FORCE0      = 2,
    CUSUM       = 3,
    ACTIVATION  = 4,
};
std::map<taxel_specifier_t, String> tactile_publisher_map {
    {FORCE, "force"},
    {RAW_SIG, "raw"},
    {FORCE0, "force0"},
    {CUSUM, "cusum"},
    {ACTIVATION, "active"},
};

const uint8_t ALL_TAXELS = 8;

enum sensor_specifier_t : size_t{
    S1=0,
    S2=1
};

struct tactile_publisher : public Publishable
{
    const static int TYPE_ID = __COUNTER__;
    const size_t sens_i;
    const uint8_t taxel;
    const taxel_specifier_t spec;
    hw::Fingers::fingers_t &fingers; // TODO pass reference to sensor instead
    tactile_publisher(size_t sensor, uint8_t taxel, taxel_specifier_t spec, hw::Fingers::fingers_t &fingers) : Publishable(TYPE_ID, (sensor<<7)|taxel<<3|spec), sens_i(sensor < 2 ? sensor : 0), taxel(taxel), spec(spec), fingers(fingers) {}

    void publish(Print &out) override {
        // Threads::Scope scope(hw::tactile_mutex);

        auto &sensor = fingers[sens_i]->TactileSensor;
        switch (spec)
        {
        case FORCE:
            if(taxel <= sensor.n_taxels()){
                sensor.printTo(out, (size_t)taxel, false);
                out<<" ";
            }else if(taxel == ALL_TAXELS){
                sensor.printTo(out, false);
            }
            break;

        case RAW_SIG:
            if(taxel <= sensor.n_taxels()){
                out << _FLOAT(1000*sensor.raw_signal(taxel),3) << " ";
            }else if(taxel == ALL_TAXELS){
                for (size_t i = 0; i < sensor.n_taxels(); i++) out << _FLOAT(1000*sensor.raw_signal(i),3) << " ";
            }
            break;

        case FORCE0:
            if(taxel <= sensor.n_taxels()){
                out << _FLOAT(sensor.force0(taxel),3) << " ";
            }else if(taxel == ALL_TAXELS){
                for (size_t i = 0; i < sensor.n_taxels(); i++) out << _FLOAT(sensor.force0(i),3) << " ";
            }
            break;
        
        case CUSUM:
            if(taxel <= sensor.n_taxels()){
                out << _FLOAT(sensor.cusum(taxel),3) << " ";
            }else if(taxel == ALL_TAXELS){
                for (size_t i = 0; i < sensor.n_taxels(); i++) out << _FLOAT(sensor.cusum(i),3) << " ";
            }
            break;

        case ACTIVATION:
            if(taxel <= sensor.n_taxels()){
                out << (sensor.is_activated(taxel)?1:0) << " ";
            }else if(taxel == ALL_TAXELS){
                for (size_t i = 0; i < sensor.n_taxels(); i++) out << (sensor.is_activated(i) ? 1:0) << " ";
            }
            break;

        }          
    }
    void printTo(Print &out) override {
        out << tactile_publisher_map[spec] << "(s"<<sens_i;
        if(taxel!=ALL_TAXELS) out << ".t"<<taxel;
        out << ")";
    }

};