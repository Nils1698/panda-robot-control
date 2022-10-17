#include <Arduino.h>
#include "Streaming.h"
#include "UserStream.h"
#include "executer.h"
#include "UserCom(deprecated)/user_com.h"

#include <SPI.h>
#include "OpticalSensor.h"
//OpticalSensor opt_sensor(10, SPI), opt_sensor2(9, SPI); // SPI0
OpticalSensor opt_sensor(43, SPI2), opt_sensor2(50, SPI2); // SPI2
time_t optsens_time_start;

String in_str;
float in_values[10];

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // SPI0
  // SPI.setSCK(13);
  // SPI.setMOSI(11);
  // SPI.setMISO(12);

  // SPI2
  // SPI.setSCK(46);
  // SPI.setMISO(45);
  // SPI.setMOSI(44);

  user_serial::begin();
  debug_stream << "SETUP - OPTICAL TEST\n";
  
  if(opt_sensor.connect()){
    debug_stream<<"sensor 1 connected\n";
  }else{
    debug_stream<<"sensor 1 connection failed!\n";
  }
  if(opt_sensor2.connect()){
    debug_stream<<"sensor 2 connected\n";
  }else{
    debug_stream<<"sensor 2 connection failed!\n";
  }
}

void loop()
{
  if (UserCom::checkSerialInput(in_str))
  {
    debug_stream << "Recieved: " << in_str << "\n";

    // TESTS //
    if(UserCom::getInput(in_str, "print", in_values[0])){
    }
    else if(in_str=="print"){
    }

    else if(in_str=="led on"){
      opt_sensor.setLed(true);
      opt_sensor2.setLed(true);
    }
    else if(in_str=="led off"){
      opt_sensor.setLed(false);
      opt_sensor2.setLed(false);
    }
    else if(in_str=="reset"){
      opt_sensor.resetSlide();
      opt_sensor2.resetSlide();
    }

    // else if(in_str=="test2"){

    //   constexpr float init_force = 1;
    //   float grip_force = init_force;

    //   executor.execute([]{
    //     controller->run(init_force, []{return executor.is_ok();});
    //   });

    //   const int T = 10;
    //   slipDetector.execute_loop([&grip_force]{
    //     opt_sensor.update();
    //     if( opt_sensor.slide() > 0.01 /*&& hw::Fingers ...*/){
    //       grip_force += 0.5;
    //       controller->set_force(grip_force);
    //     }
        

    //   }, T);
    // }

  }

// -20.04: -46
// 5.66 -953

  opt_sensor.update();
  opt_sensor2.update();
  if(millis()-optsens_time_start > 100){
    optsens_time_start = millis();
    signal_stream << "" << _FLOAT(opt_sensor.slideX(),3) << " " << _FLOAT(opt_sensor.slideY(),3);// << "\n";
    signal_stream << " " << _FLOAT(opt_sensor2.slideX(),3) << " " << _FLOAT(opt_sensor2.slideY(),3) << "\n";
    // signal_stream << _FLOAT(0.0254*opt_sensor.slideX(),3) << " " << _FLOAT(0.0254*opt_sensor.slideY(),3) << "\n";

    // signal_stream << _FLOAT(0.028366446*opt_sensor.slideX(),3) << " " << _FLOAT(0.028366446*opt_sensor.slideY(),3) << "\n";
    // signal_stream << opt_sensor.slideX() << " " << opt_sensor.slideY() << "\n";
    // debug_stream << "x: " << opt_sensor.slideX() << "y: " << opt_sensor.slideY() << "\n";

  }

}