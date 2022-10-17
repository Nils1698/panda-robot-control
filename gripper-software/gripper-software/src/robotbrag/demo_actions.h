#pragma once

/*
** Interface used for Robotbrag demo, May '22
**
*/

#include "HW_prototype.h"
#include "tactile_perception.h"
#include "demo_objects.h"

#include "UserAction.h"
#include "Listener.h"
#include "tactile_calibration.h"
#include "gripper_actions.h"

#define DONE_SIGNAL "done"

void grip_limitted(const float force = 0.5);
struct char_result {
    float centering=0;
    int obj_index=-1;
};
char_result tactile_characterize(std::vector<DemoObject*> &DemoObjects, int exp_obj=0);
void await_motion(int slide_thres, int period_ms, std::function<void()> finish_func);

extern std::vector<DemoObject*> DemoObjects;
extern int demo_object_index;
extern int slip_thres, motion_thres, lift_thres; // used in slip_grip
extern float DETECT_FORCE;

namespace UserInput {

class demo_actions : public ActionList
{
public:
  demo_actions() : ActionList("") {

                                        // Name,            width,          hardness,                   area,                 flatness,           desired gripping force
    DemoObjects.push_back(new DemoObject({"EmptyCan", DemoParam(65.7, 3), DemoParam(1.8, 1.1, 3),  DemoParam(0.7, 0.4, 1.1), DemoParam(4.5, 3.5, 16),  3}));
    DemoObjects.push_back(new DemoObject({"FullCan",  DemoParam(65.7, 3), DemoParam(5, 3, 10),     DemoParam(0.7, 0.4, 1.1), DemoParam(2, 0.5,  12),    10}));
    DemoObjects.push_back(new DemoObject({"Mandarin", DemoParam(59, 55, 61), DemoParam(2, 1.5, 4), DemoParam(0.6, 0.25, 0.9), DemoParam(3, 0,  5),    4}));
    DemoObjects.push_back(new DemoObject({"Tomato",   DemoParam(53.4, 49, 56), DemoParam(2.7, 1.6, 5.5), DemoParam(0.58, 0.25, 1.0), DemoParam(2, 0.1,  5),    4}));

    actions = {
      {"detect", {
        {"speed", Parser::parse_arg<float>({0,150, 50})}
      }, [&](UserInput::arg_list_t args){
        float spd = args[0].get_float();
        GripperActions::executer.execute([spd]{
            TactileCalibration::zcal();
            switch(GripperActions::tactile_detect(DETECT_FORCE, spd)){
            case GripperActions::DETECT_RES_F1:
                UserSerial << DONE_SIGNAL << ":f1\n";
                break;
            case GripperActions::DETECT_RES_F2:
                UserSerial << DONE_SIGNAL << ":f2\n";
                break;
            case GripperActions::DETECT_RES_BOTH:
                UserSerial << DONE_SIGNAL << ":f2\n";
                break;
            case GripperActions::DETECT_RES_NONE:
                UserSerial << DONE_SIGNAL << ":none\n";
                break;
            case GripperActions::DETECT_RES_TORQUE:
                UserSerial << DONE_SIGNAL << ":torque\n";
                break;
            case GripperActions::DETECT_RES_TIMEOUT:
                UserSerial << DONE_SIGNAL << ":timeout\n";
                break;
            }
        });
      }},

      {"char-obj", {
        {"expected object", UserInput::Parser::parse_arg<int>()}
      }, [](UserInput::arg_list_t args){
        const int exp_obj = args[0].get_int();
        GripperActions::executer.execute([exp_obj]{
          GripperActions::tactile_detect();
          GripperActions::tactile_close(1.0);
          char_result res = tactile_characterize(DemoObjects, exp_obj);
          UserSerial << DONE_SIGNAL << ":{'obj':'" << (res.obj_index < 0 ? "unknown" : DemoObjects[res.obj_index]->name) << "','cnt':" << res.centering << "}\n";
          demo_object_index = res.obj_index;
          hw::motor()->open();
        });    
      }},

      {"char", {
      }, [&](UserInput::arg_list_t args){
        GripperActions::executer.execute([]{
          char_result res = tactile_characterize(DemoObjects);
          UserSerial << DONE_SIGNAL << ":{'obj':'" << (res.obj_index < 0 ? "unknown" : DemoObjects[res.obj_index]->name) << "','cnt':" << res.centering << "}\n";
          demo_object_index = res.obj_index;
        });
      }},

      {"hold-obj", {
      }, [&](UserInput::arg_list_t args){
        if(demo_object_index>=0){
          GripperActions::executer.execute([]{
            GripperActions::hold_force(DemoObjects[demo_object_index]->grip_force);
          });
        }
      }},

      // {"grip-slip-a", {
      // }, [&](UserInput::arg_list_t args){
      //   await_motion(motion_thres, 10, []{
      //     debug_stream << "Detect!\n";
      //     user_input.set_input("grip slip");
      //   });
      // }},

      {"grip-slip", {
      }, [&](UserInput::arg_list_t args){
        delay(100);
        GripperActions::executer.execute([]{
          GripperActions::tactile_detect();
          GripperActions::init_slip_detector(slip_thres, -1, lift_thres, true, true);
          GripperActions::hold_force(1.0);
          GripperActions::stop_slip_detector();
          hw::motor()->open();
          UserSerial<<DONE_SIGNAL<<"\n";
        });
      }},

      {"grip-demo", {
      }, [&](UserInput::arg_list_t args){
        grip_limitted();
      }},

      {"slide-grip", {
        {"slip thres", UserInput::Parser::parse_arg<int>()},
        {"init force", UserInput::Parser::parse_arg<float>(1)},
        {"no slip return", UserInput::Parser::parse_arg<float>(0, 30, 0)}
      }, [&](UserInput::arg_list_t args){
        int slip_thres = args[0].get_int();
        float init_force = args[1].get_float();
        float no_slip_return = args[2].get_float();
        GripperActions::executer.execute([=]{
          float f = GripperActions::hold_detect_slip(slip_thres, init_force, no_slip_return*1000);
          UserSerial << DONE_SIGNAL << ":" << f << "\n";
        });
      }},

      {"await-motion", {
        {"motion thres", Parser::parse_arg<int>(10)}
      }, [&](UserInput::arg_list_t args){
        await_motion(args[0].get_int(), 10, []{
          UserSerial << DONE_SIGNAL << "\n";
          hw::Fingers::F[0]->opticalSensor.setLed(false);
          hw::Fingers::F[1]->opticalSensor.setLed(false);
        });
      }},


    };
  }
};

} // USerCom