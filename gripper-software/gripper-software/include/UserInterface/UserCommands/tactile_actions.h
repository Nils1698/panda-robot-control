#pragma once

#include "executer.h"
#include "fingers.h"
#include "tactile_calibration.h"
#include "tactile_perception.h"
#include "UserAction.h"

namespace UserInput {
class tactile_actions : public ActionList
{
public:
  tactile_actions(String sub) : ActionList(sub) {
    actions = {

      // PRINTING //
      {"force", {
      }, [&](UserInput::arg_list_t args){
        info_stream << hw::Fingers::total_force() << "\n";
      }},

      // CALIBRATION //
      {"save-cal", {
      }, [&](UserInput::arg_list_t args){
        TactileCalibration::cal_save();
        info_stream << "Saved\n";
      }},

      {"load-cal", {
      }, [&](UserInput::arg_list_t args){
        TactileCalibration::cal_load();
        info_stream << "Loaded\n";
      }},

      {"reset-cal", {
      }, [&](UserInput::arg_list_t args){
        TactileCalibration::cal_reset();
        info_stream << "Reset\n";
      }},

      {"get-cal", {
      }, [&](UserInput::arg_list_t args){
        for (size_t s = 0; s < 2; s++){
          auto &sensor = hw::Fingers::F[s]->TactileSensor;
          signal_stream << "Sensor "<<s+1<<"\n";
          for (size_t ti = 0; ti < sensor.n_taxels(); ti++)
          {
            signal_stream << "set-cal "<<s<<" "<<ti<<" ";
            tactile_sensor::calib_type cal = sensor.get_calib_params(ti);
            for (size_t ci = 0; ci < cal.size(); ci++)
            {
              if(ci>0) signal_stream << ",";
              float p = cal[ci];
              int ii=0;
              while(abs(p)<1.0 && p!=0 && ii<15){
                p *= 10;
                ii++;
              }
              signal_stream << _FLOAT(p,4);
              if(ii>0) signal_stream << "e-"<<ii;
            }
            signal_stream << "\n";
          }
        }
      }},

      {"set-cal", {
        {"Sensor",    Parser::parse_arg<int>({0,1})},
        {"Taxel",     Parser::parse_arg<int>({0,6})},// TODO constant
        {"Values"}
      }, [&](UserInput::arg_list_t args){
        debug_stream << "Values: "<<args[2].get_string()<<"\n";
        auto &sensor = hw::Fingers::F[args[0].get_int()]->TactileSensor;
        int taxel_i = args[1].get_int();
        UserInput::Parser p(args[2].get_string(), ',');
        tactile_sensor::calib_type cal;
        for(float &c : cal){
          c = p.parse(Parser::parse_arg<float>());
          debug_stream << c<<"\n";
        }
        if(p.verify_input()){
          sensor.set_calib_params(taxel_i, cal);
        }
      }},

      {"save-thres", {
      }, [&](UserInput::arg_list_t args){
        TactileCalibration::cal_save_thres();
        info_stream << "Saved\n";
      }},

      {"load-thres", {
      }, [&](UserInput::arg_list_t args){
        TactileCalibration::cal_load_thres();
        info_stream << "Saved\n";
      }},

      {"get-thres", {
      }, [&](UserInput::arg_list_t args){
        for (size_t s = 0; s < 2; s++){
          auto &sensor = hw::Fingers::F[s]->TactileSensor;
          signal_stream << "Sensor "<<s+1<<"\n";
          for (size_t i = 0; i < sensor.n_taxels(); i++)
          {
            signal_stream <<s+1<<" "<<i<<" " << sensor.get_threshold(i) << "\n";
          }
        }
      }},
      
      {"set-thres", {
        {"Sensor",    Parser::parse_arg<int>({0,1})},
        {"Taxel",     Parser::parse_arg<int>({0,6})},// TODO constant
        {"Threshold", Parser::parse_arg<float>()},
      }, [&](UserInput::arg_list_t args){
        auto thres = args[2].get_float();
        hw::Fingers::F[args[0].get_int()]->TactileSensor.set_threshold(args[1].get_int(), thres);
        debug_stream << "Thres: " << thres << "N\n";
      }},

      {"load-thres", {
      }, [&](UserInput::arg_list_t args){
        TactileCalibration::cal_load_thres();
        info_stream << "Saved\n";
      }},

      {"zcal", {
      }, [&](UserInput::arg_list_t args){
        TactileCalibration::zcal();
        info_stream << "Calibrated\n";
      }},

      {"reset-zcal", {
      }, [&](UserInput::arg_list_t args){
        TactileCalibration::zcal_reset();
        info_stream << "Reset\n";
      }},

    };
  }
};

//     // CALIBRATION //

//     else if(UserInput::getInputs(in_str, "calib", 2, in_values)){
//       size_t i = in_values[0]-1;
//       if(i >= 2) return false;
//       auto &sensor = hw::Fingers::F[i]->TactileSensor;

//       size_t tax = in_values[1];
//       if(tax >= 7) return false;

//       sensor.reset_calib_params(tax);

//       while(1){
//         sensor.measure();
//         sensor.printTo(signal_stream, tax);
//         if (UserInput::checkSerialInput(in_str)){
//           if(in_str=="n"){
//             if(++tax>=7) break;
//             sensor.reset_calib_params(tax);
//           }
//           else break;
//         }
//         delay(20);
//       }
//     }


} //UserInput