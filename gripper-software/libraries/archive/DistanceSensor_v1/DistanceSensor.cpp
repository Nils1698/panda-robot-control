
#include "DistanceSensor.h"

namespace distance_sensor {

DistanceSensor::DistanceSensor(TwoWire *wire, String name){
  _name = name;
  sensor = Adafruit_VL6180X();
  _connected = sensor.begin(wire);
}

DistanceSensor::DistanceSensor(TwoWire *wire, pin_t scl, pin_t sda, String name){
  _name = name;
  sensor = Adafruit_VL6180X();
  wire->setSCL(scl);
  wire->setSDA(sda);
  _connected = sensor.begin(wire);
}

bool DistanceSensor::is_ok(){
  return _connected;
}

String DistanceSensor::name(){
  return _name;
}

void DistanceSensor::measure(){
    if(millis()-timeLastUpdate >= _update_delay_ms){
      timeLastUpdate=millis();
      _lock.lock();
        _stable_value = count>0 ? (float)sum / count : -1;
        _is_stable = valid;
      _lock.unlock();

      count=0;
      sum=0;
      min_val=255;
      max_val=0;
      valid=true;
    }

    if(millis()-timeLastRead >= _read_delay_ms){
      timeLastRead=millis();
      int val = read_raw();
      if(val<0){
        valid = false;
        return;
      }
      sum += val;
      count++;

      min_val = min(min_val, val);
      max_val = max(max_val, val);
      if(max_val-min_val > stable_max_diff){
        valid = false;
      }
    }
}

void DistanceSensor::measure_thread(void *arg){
  DistanceSensor *sensor = (DistanceSensor*)arg;
  while(1){
    // sensor->read_stable();
    sensor->measure();
    threads.yield();
  }
}

void DistanceSensor::start_measuring(unsigned long update_delay, size_t meas_pr_update){
  if(thread_id == -1){
    _update_delay_ms = update_delay;
    _meas_pr_update = meas_pr_update;
    _read_delay_ms = update_delay / meas_pr_update;
    timeLastUpdate = millis();
    timeLastRead = millis();

    thread_id = threads.addThread(DistanceSensor::measure_thread,this);
  }else{
    if(threads.getState(thread_id) == threads.SUSPENDED){
      _update_delay_ms = update_delay;
      _meas_pr_update = meas_pr_update;
      _read_delay_ms = update_delay / meas_pr_update;
      timeLastUpdate = millis();
      timeLastRead = millis();

      threads.restart(thread_id);
    }else{
      Serial.println("Thread already running or ended!");
      return;
    }
  }
  
}

void DistanceSensor::stop_measuring(){
  if(thread_id == -1){
    Serial.println("No thread to stop!");
  }else{
    if(threads.getState(thread_id) == threads.RUNNING){
      threads.suspend(thread_id);
    }else{
      Serial.println("Thread not running!");
      return;
    }
  }
}

int DistanceSensor::read_raw(){
  uint8_t range  = sensor.readRange();
  uint8_t status = sensor.readRangeStatus();
  if(status==VL6180X_ERROR_NONE && range!=255){
#if DEBUG_DIST
      Serial.println(range);
#endif

    return range;
  }
  else
  {
#if DEBUG_DIST
    Serial.print("VL6180X error: ");
    Serial.println(get_error_msg(status));
#endif

    return -1;
  }

}

float DistanceSensor::read(){
  int val = read_raw();
  return val<0 ? -1 : val * c1+c0;
}

// float DistanceSensor::read_stable(const int max_diff=5, const int N=10, const int DELAY=100){
//   int min_val=255,max_val=0, sum=0, count=0;
//   bool valid=true;

//   for(int i=0; i<N; i++){
//     int val = read_raw();
//     if(val<0){
//       valid = false;
//       break;
//     }
//     min_val = min(min_val, val);
//     max_val = max(max_val, val);
    
// #if DEBUG_DIST
//     Serial << "max: " << max_val << "\tmin: "<<min_val<< "\tdiff:" << max_val-min_val << "\n";
// #endif

//     if(val < 0 || max_val-min_val > max_diff){
//       valid = false;
//       break;
//     }

//     sum += val;
//     delay(DELAY);
//   }
  
//   _lock.lock();
//   if(valid){
//     _stable_value = sum/N;
//   }else{
//     _stable_value = -1;
//   }
//   _lock.unlock();
  
//   return _stable_value;
// }

void DistanceSensor::set_calibration(float offset, float factor){
  c0 = offset;
  c1 = factor;
}

float DistanceSensor::stable_value(){
  Threads::Scope scope(_lock);
  return _stable_value<0 ? -1 : _stable_value * c1+c0;
}

bool DistanceSensor::is_stable(){
  Threads::Scope scope(_lock);
  // return _stable_value >= 0;
  return _is_stable;
}

String DistanceSensor::get_error_msg(uint8_t status){
  if(status >= VL6180X_ERROR_SYSERR_1 && status <= VL6180X_ERROR_SYSERR_5){
    return "System error";
  }
  switch(status){
    case VL6180X_ERROR_ECEFAIL:
      return "ECE failure";
    case VL6180X_ERROR_NOCONVERGE:
      return "No convergence";
    case VL6180X_ERROR_RANGEIGNORE:
      return "Ignoring range";
    case VL6180X_ERROR_SNR:
      return "Signal/Noise error";
    case VL6180X_ERROR_RAWUFLOW:
      return "Raw reading underflow";
    case VL6180X_ERROR_RAWOFLOW:
      return "Raw reading overflow";
    case VL6180X_ERROR_RANGEUFLOW:
      return "Reading underflow";
    case VL6180X_ERROR_RANGEOFLOW:
      return "Reading overflow";
  }
  return "None";
}

} // distance_sensor
