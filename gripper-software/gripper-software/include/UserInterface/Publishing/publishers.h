#include "pubsub.h"

namespace PUB {

struct control_publisher : publisher_t {
    control_publisher(ForceController* &controller){
        id = "ctrl";
        categories = {
            {"des_force", [&controller](Print& out, uint8_t spec){
                out << controller->get_desired_force() << " ";
            }},
            {"error", [&controller](Print& out, uint8_t spec){
                out << controller->get_error()  << " ";
            }},
            {"cmd_vel", [&controller](Print& out, uint8_t spec){
                out << controller->get_cmd_vel() << " ";
            }},
        };
    }
};

struct tactile_publisher : publisher_t {
    typedef tactile_sensor::RowSensor<7> sensor_t;
    const static uint8_t ALL_TAXELS = subscription::NO_SPEC;
    tactile_publisher(String t_id, sensor_t &sensor){
        id = t_id;

        categories = {
            {"force", [&sensor,this](Print& out, uint8_t taxel){
                publish_value(out, taxel, sensor, [&sensor](Print&out, size_t t){
                    out<<_FLOAT(sensor.force(t),3);
                });
            }},
            {"raw", [&sensor,this](Print& out, uint8_t taxel){
                publish_value(out, taxel, sensor, [&sensor](Print&out, size_t t){
                    out << _FLOAT(1000*sensor.raw_signal(t),3);
                });
            }},
            {"force0", [&sensor,this](Print& out, uint8_t taxel){
                publish_value(out, taxel, sensor, [&sensor](Print&out, size_t t){
                    out << _FLOAT(1000*sensor.force0(t),3);
                });
            }},
            {"cusum", [&sensor,this](Print& out, uint8_t taxel){
                publish_value(out, taxel, sensor, [&sensor](Print&out, size_t t){
                    out << _FLOAT(1000*sensor.force0(t),3);
                });
            }},
            {"act", [&sensor,this](Print& out, uint8_t taxel){
                publish_value(out, taxel, sensor, [&sensor](Print&out, size_t t){
                    out << (sensor.is_activated(t) ? 1 : 0);
                });
            }},
        };

        spec_map = {{"all", ALL_TAXELS}};
    }

private:
    void publish_value(Print& out, uint8_t taxel, sensor_t &sensor, std::function<void(Print&,size_t)> print_func){
        // Threads::Scope scope(hw::tactile_mutex);
        if(taxel <= sensor.n_taxels()){
            print_func(out, taxel);
            out << " ";
        }else if(taxel == ALL_TAXELS){
            for (size_t i = 0; i < sensor.n_taxels(); i++) {
                print_func(out, i);
                out << " ";
            };
        }
    }

};

struct motor_publisher : publisher_t
{
    motor_publisher(motor_interface::Motor &motor) {
        id = "motor";
        categories = {
            {"pos", [&motor](Print& out, uint8_t spec){
                out << motor.get_position() << " ";
            }},
            {"vel", [&motor](Print& out, uint8_t spec){
                out << motor.get_velocity() << " ";
            }},
            {"torque", [&motor](Print& out, uint8_t spec){
                out << motor.get_torque_raw() << " ";
            }},
        };
    }
};

struct optical_publisher : publisher_t
{
    optical_publisher(String sensor_id, OpticalSensor &opt) {
        id = sensor_id;
        categories = {
            {"X", [&opt](Print& out, uint8_t spec){
                out << opt.deltaX() << " ";
            }},
            {"Y", [&opt](Print& out, uint8_t spec){
                out << opt.deltaY() << " ";
            }},
        };
    }
};

struct perception_publisher : publisher_t
{
    // TODO make perception class and move computations away from publish functions
    perception_publisher(hw::Fingers::fingers_t &fingers) {
        id = "per";
        categories = {
            {"force", [&fingers](Print& out, uint8_t spec){
                // Threads::Scope scope(hw::tactile_mutex);
                out << fingers[0]->TactileSensor.total_force() << " "
                    << fingers[1]->TactileSensor.total_force() << " ";
            }},
            {"tau", [&fingers](Print& out, uint8_t spec){
                // Threads::Scope scope(hw::tactile_mutex);
                TactilePerception::contact_info_t ci0 = TactilePerception::compute_contact_info( fingers[0]->TactileSensor.forces(), 0.1, 0.7);
                TactilePerception::contact_info_t ci1 = TactilePerception::compute_contact_info( fingers[1]->TactileSensor.forces(), 0.1, 0.7);
                TactilePerception::wrench_info_t wrench = TactilePerception::compute_wrench(ci0, ci1);                

                out << wrench.tau_x << " " << wrench.tau_z << " ";
            }},
            {"f_net", [&fingers](Print& out, uint8_t spec){
                // Threads::Scope scope(hw::tactile_mutex);
                TactilePerception::contact_info_t ci0 = TactilePerception::compute_contact_info( fingers[0]->TactileSensor.forces(), 0.1, 0.7);
                TactilePerception::contact_info_t ci1 = TactilePerception::compute_contact_info( fingers[1]->TactileSensor.forces(), 0.1, 0.7);
                TactilePerception::wrench_info_t wrench = TactilePerception::compute_wrench(ci0, ci1);                

                out << wrench.f_y << " ";
            }},
            {"CoM", [&fingers](Print& out, uint8_t spec){
                // Threads::Scope scope(hw::tactile_mutex);
                TactilePerception::contact_info_t ci0 = TactilePerception::compute_contact_info( fingers[0]->TactileSensor.forces(), 0.1, 0.7);
                TactilePerception::contact_info_t ci1 = TactilePerception::compute_contact_info( fingers[1]->TactileSensor.forces(), 0.1, 0.7);

                out << ci0.cm_x << " " << ci1.cm_x << " " << ci0.cm_y << " " << ci1.cm_y << " ";
            }},
            {"contacts", [&fingers](Print& out, uint8_t spec){
                // Threads::Scope scope(hw::tactile_mutex);
                out << TactilePerception::compute_contact_points(fingers[0]) << " "
                    << TactilePerception::compute_contact_points(fingers[1]) << " ";
            }},
            {"cusum", [&fingers](Print& out, uint8_t spec){
                // Threads::Scope scope(hw::tactile_mutex);
                for (size_t s = 0; s < 2; s++){
                    float cusum=0;
                    for (size_t i = 0; i < fingers[s]->TactileSensor.n_taxels(); i++){
                        cusum+=fingers[s]->TactileSensor.cusum(i);
                    }                
                    out << _FLOAT(cusum,4) << " ";
                }
            }},
        };
    }
};


struct system_publisher : publisher_t
{
    system_publisher() {
        id = "sys";
        categories = {
            {"ms", [](Print& out, uint8_t spec){
                out << millis() << " ";
            }},
            {"us", [](Print& out, uint8_t spec){
                out << micros() << " ";
            }},
        };
    }
};

} // PUB