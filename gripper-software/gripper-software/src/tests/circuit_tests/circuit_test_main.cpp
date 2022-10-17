#include <Arduino.h>
#include "Streaming.h"
#include "TeensyThreads.h"

#include "executer.h"
#include "UserStream.h"
#include "UserCom(deprecated)/user_com.h"

#include "CircuitBoard.h"

#include <array>

exec::Executer executor;
String in_str;
float in_values[10];

std::array<float,7> RES_BANK = {10, 56, 100, 750, 1000, 2200, 5600};

void print_taxels(tactile_sensor::Circuit_2x7 &circ, bool endline=true){
    for (size_t i = 0; i < 7; i++){
        signal_stream << _FLOAT(1000*circ.measure_conductance(i),5) << " ";    
    }
    if(endline) signal_stream << "\n";    
}

void print_taxels_res(tactile_sensor::Circuit_2x7 &circ, bool endline=true){
    for (size_t i = 0; i < 7; i++){
        signal_stream << _FLOAT(1/circ.measure_conductance(i),5) << " ";    
    }
    if(endline) signal_stream << "\n";    
}

using namespace hw;

void setup()
{
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    UserSerial.begin(500000);
    while(!UserSerial){delay(1);}
    debug_stream << "SETUP - CIRCUIT TESTS\n";
    debug_stream << "CIRCUIT VERSION "<<CIRCUIT_VERSION<<"\n";

    hw::analog::init();
    circI.init();
    circII.init();
    circI.set_DAC_delay(200);
    circII.set_DAC_delay(200);
    circI.set_useAutoRange(true);
    circII.set_useAutoRange(true);

    delay(500);
    debug_com_stream.setActive(false);
    debug_stream.setActive(true);
}

void loop()
{
  if (UserCom::checkSerialInput(in_str))
  {
    debug_stream << "Recieved: " << in_str << "\n";

    // TESTS //
    if(UserCom::getInput(in_str, "print", in_values[0])){
        if(in_values[0]==1){
            print_taxels(circI);
        }else if(in_values[0]==2){
            print_taxels(circII);
        }
    }
    else if(in_str=="print"){
        print_taxels(circI,false);
        print_taxels(circII);
    }
    else if(in_str=="print res"){
        print_taxels_res(circI,false);
        print_taxels_res(circII);
    }
    else if(UserCom::getInput(in_str, "stream", in_values[0])){
        unsigned long last_sample_time=0;  
        while(!UserCom::checkSerialInput(in_str)){
            if(millis()-last_sample_time > 100){
                last_sample_time = millis();
                print_taxels(in_values[0]==1 ? circI: circII);
            }
            delay(20);
        }
    }
    else if(UserCom::getInput(in_str, "stream res", in_values[0])){
        unsigned long last_sample_time=0;  
        while(!UserCom::checkSerialInput(in_str)){
            if(millis()-last_sample_time > 100){
                last_sample_time = millis();
                print_taxels_res(in_values[0]==1 ? circI: circII);
            }
            delay(20);
        }
    }

    else if(UserCom::getInputs(in_str, "test mux", 1, in_values)){
        tactile_sensor::Circuit_2x7 *circ = in_values[0]==1 ? &circI : &circII;
        circ->check_reference();
        for (size_t i = 0; i < 8; i++) info_stream << "t" << i << "\t";
        info_stream << "\n";
        for (size_t i = 0; i < 8; i++){
            circ->test_order(i);
        }
    }

    else if(UserCom::getInput(in_str, "test", in_values[0])){
        tactile_sensor::Circuit_2x7 *circ = in_values[0]==1 ? &circI : &circII;

        info_stream << "Res\tG[uS]\tGm[uS]\tErr\n";

        for (size_t i = 0; i < circ->taxel_limit(); i++)
        {
            const int N_SAMPLES = 100;
            float G_sum=0;
            for (int s = 0; s < N_SAMPLES; s++)
            {
                G_sum += circ->measure_conductance(i); 
                delay(10);
            }
            const float G_true = 1./RES_BANK[i];
            const float G_mean = G_sum/N_SAMPLES;
            float rel_err = (G_mean - G_true)/G_true;
            info_stream << RES_BANK[i] << "\t";
            info_stream << 1000*G_true << "\t";
            info_stream << 1000*G_mean << "\t";
            info_stream << rel_err << "\t";
            info_stream << "\n";
        }
    }
    else if(UserCom::getInputs(in_str, "set mux", 2, in_values)){
        if(in_values[0]<=2){
            tactile_sensor::Circuit_2x7 *circ = in_values[0]==1 ? &circI : &circII;
            debug_stream << "Set mux" << (in_values[0]==1?"I":"II") << ": NO" << (int) in_values[1] << "\n";
            circ->set_channel(in_values[1]);
        }
    }

    else if(UserCom::getInputs(in_str, "find order", 1, in_values)){
        if(in_values[0]<=2){
            tactile_sensor::Circuit_2x7 *circ = in_values[0]==1 ? &circI : &circII;
            circ->find_order();
        }
    }

    else if(UserCom::getInput(in_str, "test delay", in_values[0])){
        tactile_sensor::Circuit_2x7 *circ = in_values[0]==1 ? &circI : &circII;

        signal_stream << "del\\res\t";
        for (size_t i = 0; i < RES_BANK.size(); i++)
            info_stream<< RES_BANK[i]<<"\t";
        signal_stream << "\n";

        std::array<int, 5> DELAYS_US = {10, 50, 100, 200, 500};
        for (const int d : DELAYS_US){
            circ->set_DAC_delay(d);
            delay(100);
            signal_stream << d << "\t";
            for (size_t i = 0; i < circ->taxel_limit(); i++)
            {
                const int N_SAMPLES = 100;
                float G_sum=0;
                for (int s = 0; s < N_SAMPLES; s++)
                {
                    G_sum += circ->measure_conductance(i); 
                    delay(10);
                }
                const float G_true = 1./RES_BANK[i];
                const float G_mean = G_sum/N_SAMPLES;
                float rel_err = (G_mean - G_true)/G_true;
                signal_stream << _FLOAT(rel_err,4) << "\t";
            }
            signal_stream << "\n";
        }
    }

  }

  delay(1);
}