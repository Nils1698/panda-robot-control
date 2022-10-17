
#include "HW_prototype.h"

#include "gripper_actions.h"
#include "executer.h"
#include "tactile_perception.h"
#include "tactile_calibration.h"

namespace GripperActions{

exec::Executer executer("GripperExecutor",{hw::tactile_mutex}), slipDetector("SlipDetector");

PID_Controller pid_controller(hw::motor(), hw::Fingers::F);
Detect_Controller detect_controller(hw::motor(), hw::Fingers::F);
ForceController * controller = &pid_controller;

void tactile_release(const float force, float speed, float max_change){
    float pos_init = hw::motor()->read_position();
    hw::motor()->velocity_control_init(speed);
    while(executer.is_ok())
    {
        if(max_change>0){
            if(hw::motor()->read_position()-pos_init >= max_change) break;
        }
        hw::Fingers::measure_tactile();
        float total_force = hw::Fingers::F[0]->TactileSensor.total_force() + hw::Fingers::F[1]->TactileSensor.total_force();
        if(total_force <= force){
            break;
        }
    }
    hw::motor()->velocity_control_set(0);
}

detetion_result tactile_detect(float detection_force, float speed)
{
    const int16_t DETECTION_TORQUE = 400; // raw
    debug_stream << "Start detection\n";

    detetion_result result = DETECT_RES_NONE;

    constexpr int DETECTIONS_NEEDED = 3;
    float init_pos = hw::motor()->read_position();
    float timeout_ms = 1.5 * init_pos / speed * 1000;
    int detect_count[] = {0, 0};
    float last_tact_force[] = {0, 0};
    // float timeout_ms = 3000;
    TactileCalibration::zcal();
    hw::motor()->velocity_control_init(-speed);
    unsigned long timeStart = millis();
    while (executer.is_ok())
    {
        if(millis() - timeStart > timeout_ms){
            result = DETECT_RES_TIMEOUT;
            break;
        }

        float mot_torque = hw::motor()->read_torque_raw();
        float mot_pos = hw::motor()->read_position();
        if(mot_pos < 1){
            result = DETECT_RES_NONE;
            break;
        }

        hw::measure_tactile();

        for (size_t i = 0; i < 2; i++)
        {
            auto f = hw::Fingers::F[i];
            float tact_force = f->TactileSensor.total_force();
            if (tact_force > detection_force)
            {
                if (tact_force > last_tact_force[i])
                {
                    detect_count[i] += 1;
                }
                else
                {
                    detect_count[i] -= 1;
                    if (detect_count[i] <= 0)
                    {
                        detect_count[i] = 0;
                        last_tact_force[i] = 0;
                    }
                }
                debug_stream << "Force detected: " << tact_force << "N\n";
                if (detect_count[i] > DETECTIONS_NEEDED)
                {
                    result = i == 0 ? DETECT_RES_F1 : DETECT_RES_F2;
                    break;
                }
                last_tact_force[i] = tact_force;
            }
        }
        if(result != DETECT_RES_NONE) break;

        if (millis() - timeStart > 1000 && fabs(mot_torque) > DETECTION_TORQUE)
        {
            debug_stream << "Torque detected: " << mot_torque << "\n";
            result = DETECT_RES_TORQUE;
            break;
        }
    }
    hw::motor()->velocity_control_set(0);
    debug_stream << "detection done\n";
    return result;
}

void tactile_close(float force, float stop_force){
    controller = &detect_controller;
    controller->run(force, 
    [stop_force]{ // while:
      return hw::Fingers::F[0]->TactileSensor.total_force() < stop_force
          || hw::Fingers::F[1]->TactileSensor.total_force() < stop_force;
    });
}

float tactile_squeeze(){
  constexpr float speed = 5, max_squeeze=3, max_force=15;
  float init_pos = hw::motor()->read_position();
  constexpr unsigned int timeout_ms = max_squeeze/speed * 1.5 * 1000;

  hw::motor()->velocity_control_init(-speed);
  unsigned long timeStart = millis();
  float pos, force=0, squeeze=-1;
  while(millis() - timeStart < timeout_ms){
    hw::measure_tactile();
    force = hw::current_force();
    pos = hw::motor()->read_position();
    squeeze = fabs(pos-init_pos);
    // debug_stream << "pos: " << pos << "mm, " << squeeze << "mm / force: " << force<< "N\n";
    if(squeeze>=max_squeeze || force>=max_force){
      debug_stream << "break at squeeze: " << squeeze << "mm / force: " << force<< "N\n";
      debug_stream << "Hardness: " << force/squeeze << "N/mm\n";
      break;
    }
  }
  hw::motor()->velocity_control_set(0);
  return force>0 ? force/squeeze : 0;
}

void init_slip_detector(int slip_thres, unsigned int no_slip_return, int lift_thres, const bool decrease_grip, bool upside_down){
    const int T = 10;
    slipDetector.execute([slip_thres, lift_thres, decrease_grip, upside_down, no_slip_return]{
        auto &opt_sensor1 = hw::Fingers::F[0]->opticalSensor;
        auto &opt_sensor2 = hw::Fingers::F[1]->opticalSensor;

        hw::Fingers::F[0]->opticalSensor.setLed(true);
        hw::Fingers::F[1]->opticalSensor.setLed(true);

        // Wait for controller to start
        while(slipDetector.is_ok() && !controller->is_active()){
            threads.yield();
        }

        threads.delay(1000);

        opt_sensor1.update();
        opt_sensor2.update();
        opt_sensor1.resetSlide();
        opt_sensor2.resetSlide();

        unsigned long time_last = 0;
        unsigned long last_slip = millis();
        float slide=0;
        while(slipDetector.is_ok()){
            if(no_slip_return>0 && millis()-last_slip > no_slip_return){
                debug_stream << "No slip in " << millis()-last_slip << "ms\n";
                break;
            }
            if(millis()-time_last > T){
                time_last = millis();

                opt_sensor1.update();
                opt_sensor2.update();

                // float slide = - (opt_sensor1.slideX() - opt_sensor2.slideX());
                // if(upside_down) slide = -slide;
                float delta = - (opt_sensor1.deltaX() - opt_sensor2.deltaX());
                if(upside_down) delta = -delta;

                constexpr float alph = 0.8;
                slide = alph*slide + (1-alph)*delta;

                // debug_stream << "slide: "<< slide << "\n";
                // signal_stream << slide << "\n";

                float grip_force = controller->get_desired_force();
                if( slide < -slip_thres){
                    grip_force = max(grip_force, hw::current_force() + 3);
                    debug_stream << "Increase force: "<< grip_force <<"N\n";
                    if(grip_force > 0 && grip_force < 20){
                        controller->set_force(grip_force);
                        last_slip = millis();
                    }
                }
                else if(lift_thres>0 && slide > lift_thres){
                    debug_stream << "LIFT!\n";
                    executer.request_stop();
                }
                else if(decrease_grip) {
                    grip_force = max(1, grip_force-0.01);
                    controller->set_force(grip_force);
                }
                // opt_sensor1.resetSlide();
                // opt_sensor2.resetSlide();

            }
            threads.yield();
        }
        hw::Fingers::F[0]->opticalSensor.setLed(false);
        hw::Fingers::F[1]->opticalSensor.setLed(false);
        debug_stream << "SlipDetector ended\n";
    });
}
void stop_slip_detector(){
    slipDetector.request_stop();
}

float hold_detect_slip(int slip_thres, float init_force, unsigned int no_slip_return){
    init_slip_detector(slip_thres, no_slip_return);
    controller = &pid_controller;
    controller->run(init_force, []{
        return GripperActions::executer.is_ok() && slipDetector.is_executing();
    }); // note: blocking
    // hold_force(init_force);
    slipDetector.request_stop();
    return controller->get_desired_force();
}

void hold_force(float force){
    controller = &pid_controller;
    controller->run(force, []{return GripperActions::executer.is_ok();}); // note: blocking

    // controller = &pid_controller;
    // controller->run(init_force, []{
    //   // TODO detects drop instantly!
    //   return GripperActions::executer.is_ok();
    //   // bool no_touch = hw::Fingers::F[0]->TactileSensor.total_force()<=0.05 || hw::Fingers::F[0]->TactileSensor.total_force()<=0.05;
    //   // if(no_touch || executor.is_ok()==false){
    //   //   debug_stream << "dropped!\n";
    //   //   UserSerial << "obj:drop\n";
    //   //   return false;
    //   // }
    //   // return true;
    //   // return executor.is_ok() /*&& hw::motor()->read_position()>30*/;
    // });

}

} // GripperActions