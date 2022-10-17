#pragma once

#include "Arduino.h"
#include "publishable.h"
#include "fingers.h"
#include "tactile_perception.h"
#include "TactilePerception.h"

enum perception_specifier_t : uint8_t {
    TOTAL_FORCE,
    TAU,
    F_NET,
    CoM,
    CONTACT_POINTS,
    COMB_CUSUM,
};

// TODO inclose in namespace
std::map<perception_specifier_t, String> perception_publisher_map {
    {TOTAL_FORCE, "force"},
    {TAU, "tau"},
    {F_NET, "Fnet"},
    {CoM,   "CoM"},
    {CONTACT_POINTS,   "contacts"},
    {COMB_CUSUM,   "cusum"},
};

struct perception_publisher : public Publishable
{
    const static int TYPE_ID = __COUNTER__;
    const perception_specifier_t spec;
    hw::Fingers::fingers_t &fingers;
    perception_publisher(perception_specifier_t spec, hw::Fingers::fingers_t &fingers) : Publishable(TYPE_ID, spec), spec(spec), fingers(fingers) {}

    void publish(Print &out) override {
        // Threads::Scope scope(hw::tactile_mutex);

        // TODO don't compute everything (and not here)
        TactilePerception::contact_info_t ci0 = TactilePerception::compute_contact_info( fingers[0]->TactileSensor.forces(), 0.1, 0.7);
        TactilePerception::contact_info_t ci1 = TactilePerception::compute_contact_info( fingers[1]->TactileSensor.forces(), 0.1, 0.7);
        TactilePerception::wrench_info_t wrench = TactilePerception::compute_wrench(ci0, ci1);

        switch(spec)
        {
        case FORCE:
            out << fingers[0]->TactileSensor.total_force() << " "
                << fingers[1]->TactileSensor.total_force() << " ";
            break;
        case TAU:{
            out << wrench.tau_x << " " << wrench.tau_z << " ";
            break;
        }
        case F_NET:{
            out << wrench.f_y << " ";
            break;
        }
        case CoM:
            out << ci0.cm_x << " " << ci1.cm_x << " " << ci0.cm_y << " " << ci1.cm_y << " ";
            break;
        case CONTACT_POINTS:
            out << TactilePerception::compute_contact_points(fingers[0]) << " "
                << TactilePerception::compute_contact_points(fingers[1]) << " ";
            break;
        case COMB_CUSUM:
            for (size_t s = 0; s < 2; s++){
                float cusum=0;
                for (size_t i = 0; i < fingers[s]->TactileSensor.n_taxels(); i++){
                    cusum+=fingers[s]->TactileSensor.cusum(i);
                }                
                out << _FLOAT(cusum,4) << " ";
            }
            break;
        }

    }
    void printTo(Print &out) override {
        out << perception_publisher_map[spec] << " ";
    }
};