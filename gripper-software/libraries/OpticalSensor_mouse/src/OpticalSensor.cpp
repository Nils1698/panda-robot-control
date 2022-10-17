
#include "OpticalSensor.h"
#include "UserStream.h"

USBHost myusb;
USBHIDParser hid1(myusb);
MouseController mouse1(myusb);

USBHIDInput *hiddrivers[] = {&mouse1};
#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char * hid_driver_names[CNT_HIDDEVICES] = {"Mouse1"};
bool hid_driver_active[CNT_HIDDEVICES] = {false};


bool OpticalSensor::connect(bool keep_on){
  myusb.begin(); powered_on = true;
  time_t timeStart = millis();
  const time_t CONNECTION_TIMEOUT = 2000;
  while(millis()- timeStart < CONNECTION_TIMEOUT){
    myusb.Task();

    if((USBHIDInput)mouse1){
      debug_stream << "Mouse connected!\n";
      if(!keep_on) power_off();
      return true;
    }

  }
  warning_stream << "Optical sensor conneciton timeout!\n";
  power_off();
  return false;
}

#if defined(__MK66FX1M0__)
void OpticalSensor::power_on(){
  if(powered_on) return;
  PORTE_PCR6 = PORT_PCR_MUX(1);
  GPIOE_PDDR |= (1<<6);
  GPIOE_PSOR = (1<<6); // turn on USB host power
  delay(10);
  powered_on=true;  
}
void OpticalSensor::power_off(){
  if(!powered_on) return;
  PORTE_PCR6 = PORT_PCR_MUX(1);
	GPIOE_PDDR |= (1<<6);
  GPIOE_PCOR = (1<<6); // turn off USB host power
  delay(10);
  powered_on=false;  
}
#endif

bool OpticalSensor::is_on(){
  return powered_on;
}

void OpticalSensor::update(){
  if(!powered_on) return;

  // myusb.Task(); // seems to be unnecesarry, can't find any implementation of the method

  if(mouse1.available()) {
    // Serial.print("X = ");
    // Serial.print(mouse1.getMouseX());
    // Serial.print(", Y = ");
    // Serial.print(mouse1.getMouseY());
    // Serial.println();
    
    sumX += mouse1.getMouseX();
    abssumX += abs(mouse1.getMouseX()); 
    sumY += mouse1.getMouseY();
    abssumY += abs(mouse1.getMouseY());

    mouse1.mouseDataClear();
  }
}
void OpticalSensor::set_conversion(float cx, float cy){
  conv_factor_x = cx;
  conv_factor_y = cy > 0 ? cy : conv_factor_x;
}

void OpticalSensor::resetSlide(){
  sumX=0;
  sumY=0;
  abssumX=0;
  abssumY=0;
}
float OpticalSensor::slideX(bool abs){
  return (abs ? abssumX : sumX) * conv_factor_x;
}
float OpticalSensor::slideY(bool abs){
  return (abs ? abssumY : sumY) * conv_factor_y;
}
float OpticalSensor::slide(bool abs){
  return sqrt(pow(slideX(abs),2) + pow(slideY(abs),2));
}