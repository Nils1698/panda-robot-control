#include <Arduino.h>
#include <Streaming.h>
#include "TeensyThreads.h"

#include <hw_analog_definitions.h>

#include "CircuitBoard.h"
#include <array>

using namespace hw;

String in_str;
double in_value;

constexpr size_t MAX_BYTES = 100;
char input_buffer[MAX_BYTES];
char in_byte;
int buf_i = 0;

constexpr float V_INIT_OHMITE = 0.02;
constexpr float V_INIT_OURS = 0.2;

bool checkSerialInput(String &in_str)
{
  while (Serial.available() > 0)
  {
    in_byte = Serial.read();
    input_buffer[buf_i++] = in_byte;
    if (in_byte == '\n')
    {
      input_buffer[buf_i] = '\0';
      in_str = String(input_buffer);
      in_str.replace("\r", "");
      in_str.replace("\n", "");
      buf_i = 0;
      return true;
    }
    else if (buf_i == MAX_BYTES - 1)
    {
      Serial << "Warning input buffer overflow!\n"; // debug
      buf_i = 0;
    }
  }
  return false;
}

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Serial.begin(500000);
  while (!Serial)
  {
    delay(1);
  }
  Serial << "SETUP" << "\n";
  Serial << "CIRCUIT VERSION "<<CIRCUIT_VERSION<<"\n";
  hw::analog::init();
  circI.init();
  circII.init();
  circI.set_DAC_delay(200);
  circII.set_DAC_delay(200);
  circI.set_useAutoRange(true);
  circII.set_useAutoRange(true);
  
  circII.set_VInit(V_INIT_OURS);

  // circI.set_FilterCoefs(0.8817, 0.0591, true);
  // circII.set_FilterCoefs(0.8817, 0.0591, true);

  // circI.set_FilterCoefs(0.1116, 0.7767, true);  // Ts = 20ms
  // circII.set_FilterCoefs(0.1116, 0.7767, true);  // Ts = 20ms
}

void loop_print_single(tactile_sensor::Circuit_2x7 &circ, size_t taxel_i, unsigned long T){
  unsigned long last_sample_time=0;
  while(1){
    if(millis()-last_sample_time > T){
      last_sample_time = millis();
      Serial << _FLOAT(1000*circ.measure_conductance(taxel_i),5) << "\n";    
      // digitalWrite(11,HIGH);
      // int touch = touchRead(15);
      // digitalWrite(11,LOW);
      // Serial << " "<< touch << "\n";

    }
  }
}

unsigned long last_sample_time=0;  
void print_all(tactile_sensor::Circuit_2x7 &circ, unsigned long T){
  if(millis()-last_sample_time > T){
    last_sample_time = millis();
    for (size_t i = 0; i < 6; i++){
        // Serial << 1.0/circ.measure_conductance(i) << " ";
        Serial << _FLOAT(1000*circ.measure_conductance(i),5) << ", ";    
    }
    Serial << _FLOAT(1000*circ.measure_conductance(6),5) << "\n";    
    // Serial << "\n";    
  }
}

void loop_print_all(tactile_sensor::Circuit_2x7 &circ, unsigned long T){
  unsigned long last_sample_time=0;  
  while(1){
    if(millis()-last_sample_time > T){
      last_sample_time = millis();
      for (size_t i = 0; i < 6; i++){
          // Serial << 1.0/circ.measure_conductance(i) << " ";
          Serial << _FLOAT(1000*circ.measure_conductance(i),5) << ", ";    
      }
      Serial << _FLOAT(1000*circ.measure_conductance(6),5) << "\n";    
      // Serial << "\n";    
    }
  }
}

void loop()
{
  print_all(circII, 100);
  if(checkSerialInput(in_str)){
    Serial << "Recieved: " << in_str << "\n";
    if(in_str.toLowerCase()=="ohmite"){
      Serial << "<<OHMITE>>\n";
      delay(500);
      circII.set_VInit(V_INIT_OHMITE);
    }else if(in_str.toLowerCase()=="ours"){
      circII.set_VInit(V_INIT_OURS);
      Serial << "<<OURS>>\n";
      delay(500);
    }
  }
}