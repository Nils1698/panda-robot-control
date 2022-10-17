#include "demo_actions.h"

int demo_object_index = -1;
int slip_thres=3, motion_thres=20, lift_thres=8; // used in slip_grip
std::vector<DemoObject*> DemoObjects;
float DETECT_FORCE = 0.1;

void grip_limitted(const float force){
  hw::motor()->limit_grip(30);
  GripperActions::executer.execute([force]()
  {
    TactileCalibration::zcal();
    const float spd = min(100, 50 + 10*force);
    GripperActions::tactile_detect(max(DETECT_FORCE, force/2-2), spd);
    debug_stream << "Detected! Start control..\n";
    debug_stream << "Start Control\n";
    GripperActions::tactile_close(force, 100);

    hw::motor()->unlimit_grip();
  });
}

char_result tactile_characterize(std::vector<DemoObject*> &DemoObjects, int exp_obj)
{
    float obj_width, obj_hardness, obj_area, obj_flatness;
    char_result res;

    GripperActions::tactile_release(0.6, 4, 8);

    // Set contact width = current pos
    obj_width = hw::motor()->read_position();
    debug_stream << "Width: " << obj_width << "mm\n";

    debug_stream << "Start squeeze .. \n";
    // Goto force > MAX_F || current pos > MAX_DX
    hw::led()->blink(100);
    obj_hardness = GripperActions::tactile_squeeze();
    debug_stream << "Squeeze done\n";

    // Compute hardness

    debug_stream << "Grip done\n";
    debug_stream << "\n";
    debug_stream << "Width: " << obj_width << "mm\n";
    debug_stream << "Hardness: " << obj_hardness << "N/mm\n";

    hw::Fingers::measure_tactile();
    TactilePerception::contact_info_t ci0 = TactilePerception::compute_contact_info(hw::Fingers::F[0]->TactileSensor.forces(), 0.1, 0.7);
    TactilePerception::contact_info_t ci1 = TactilePerception::compute_contact_info(hw::Fingers::F[1]->TactileSensor.forces(), 0.1, 0.7);
    debug_stream << "Area:\t" << ci0.area << "\t/\t" << ci1.area << "\n";
    debug_stream << "Cx:\t" << ci0.cm_x << "\t/\t" << ci1.cm_x << "\n";
    debug_stream << "Cy:\t" << ci0.cm_y << "\t/\t" << ci1.cm_y << "\n";
    obj_area = ci0.area + ci1.area;

    float var0 = TactilePerception::sensor_var(hw::Fingers::F[0]->TactileSensor);
    float var1 = TactilePerception::sensor_var(hw::Fingers::F[1]->TactileSensor);
    obj_flatness = (var0 + var1) / 10;
    debug_stream << "Var:\t" << var0 << "\t/\t" << var1 << "\n";

    signal_stream << exp_obj << " "
                    << obj_width << " "
                    << obj_hardness << " "
                    << obj_area << " "
                    << obj_flatness << " "
                    << "\n";

    float y_mean = (ci0.cm_y + ci1.cm_y) / 2;
    constexpr float FINGER_LENGTH = 35; // mm
    res.centering = y_mean - (FINGER_LENGTH / 2);
    // if (y_mean < 5 || y_mean > 25)
    // {
    //     debug_stream << "OBJECT NOT CENTERED!\n";
    //     hw::motor()->go_to_spd(10, 40, true, true);
    //     UserSerial << "cnt:" << res.centering << "\n";
    //     // UserSerial << DONE_SIGNAL << "\n";
    //     res.obj_index = -2;
    //     return res;
    // }

    float min_score = 1000000;
    // DemoObject *best_obj = nullptr;
    for (size_t i = 0; i < DemoObjects.size(); i++)
    {
        DemoObject * obj = DemoObjects[i];

        float score = 10000;
        debug_stream << obj->name << ": ";

        if (obj->width.matches(obj_width))
            debug_stream << "macthes width ";

        if (obj->hardness.matches(obj_hardness))
            debug_stream << "macthes hardness ";

        if (obj->area.matches(obj_area))
            debug_stream << "macthes area ";

        if (obj->flatness.matches(obj_flatness))
            debug_stream << "macthes flatness ";

        debug_stream << "\n";

        if (obj->width.matches(obj_width) && obj->hardness.matches(obj_hardness) && obj->area.matches(obj_area) /*&& obj->flatness.matches(obj_flatness)*/)
        {
            score = obj->width.residual(obj_width) + obj->hardness.residual(obj_hardness);
            if (score < min_score)
            {
                min_score = score;
                // best_obj = &obj;
                res.obj_index = i;
            }
        }
    }
    return res;

    // if (best_obj != nullptr)
    // {
    //     // UserSerial << "obj:" << (best_obj->name) << "\n";
    //     info_stream << "Object is " << (best_obj->name) << " (score: " << min_score << ")\n";
    //     return object_index;

    //     // controller = &pid_controller;
    //     // controller->run(best_obj->grip_force, [&]
    //     // {
    //     //     float pos = hw::motor()->read_position();
    //     //     //bool no_touch = hw::Fingers::F[0]->TactileSensor.total_force()<=0.05 || hw::Fingers::F[0]->TactileSensor.total_force()<=0.05;
    //     //     if(/*no_touch ||*/ pos < obj_width-5){
    //     //         debug_stream << "dropped! grip: "<<pos << "mm, obj: " << obj_width << "mm\n";
    //     //         UserSerial << "obj:drop\n";
    //     //         return false;
    //     //     }
    //     //     return true;
    //     // });
    // }
    // else
    // {
    //     // hw::motor()->open();
    //     // threads.delay(1000);
    //     // // UserSerial << "obj:unknown\n";
    //     debug_stream << "No matching object!\n";
    //     return -1;
    // }
    // hw::motor()->open();
}

void await_motion(int slide_thres, int period_ms, std::function<void()> finish_func){
  hw::Fingers::F[0]->opticalSensor.setLed(true);
  hw::Fingers::F[1]->opticalSensor.setLed(true);
  unsigned long time_start = millis();
  GripperActions::executer.execute_loop([time_start, slide_thres, finish_func]{
    auto &opt_sensor1 = hw::Fingers::F[0]->opticalSensor;
    auto &opt_sensor2 = hw::Fingers::F[1]->opticalSensor;
    opt_sensor1.resetSlide();
    opt_sensor2.resetSlide();
    opt_sensor1.update();
    opt_sensor2.update();

    float slide = opt_sensor1.slide()+opt_sensor2.slide();
    // debug_stream << "slide: " << slide << "\n"; 
    if(slide > slide_thres && millis()-time_start>1000){
      finish_func();
      GripperActions::executer.request_stop();
    }
  }, period_ms);
  
}