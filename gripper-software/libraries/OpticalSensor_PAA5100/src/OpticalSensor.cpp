
#include "OpticalSensor.h"

#include "UserStream.h"

bool OpticalSensor::connect(){
  if (!begin()) {
    error_stream << "Initialization of the flow sensor failed\n";
    return false;
  }
  _is_connected=true;
  return true;
}

bool OpticalSensor::is_connected(){
  return _is_connected;
}

void OpticalSensor::update(){
  if(!_is_connected) return;
  
  int16_t deltaX, deltaY;
  readMotionCount(&deltaX, &deltaY);
  curX = deltaX;
  curY = deltaY;

  sumX += deltaX;
  abssumX += abs(deltaX); 
  sumY += deltaY;
  abssumY += abs(deltaY);
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
float OpticalSensor::deltaX(){
  return curX * conv_factor_x;
}
float OpticalSensor::deltaY(){
  return curY * conv_factor_y;
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