
#pragma once 

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include "TeensyThreads.h"
#include "Streaming.h"
#include "hw_analog_definitions.h"

#define DEBUG_DIST 0

namespace distance_sensor {

class DistanceSensor{
  public:
    DistanceSensor(TwoWire *wire, String name = "");
    DistanceSensor(TwoWire *wire, pin_t scl, pin_t sda, String name="");
    float read();
    // float read_stable(const int max_diff=5, const int N=10, const int DELAY=100);
    void start_measuring(unsigned long update_delay, size_t meas_pr_update);
    void stop_measuring();
    bool is_stable();
    bool is_ok();
    String name();
    float stable_value();
    void set_calibration(float offset, float factor);

    static String get_error_msg(uint8_t status);

   private:
    int read_raw();
    void measure();
    static void measure_thread(void* arg);

    unsigned long _update_delay_ms = -1, _read_delay_ms = -1;
    size_t _meas_pr_update = 1;
    unsigned int stable_max_diff = 5;
    unsigned int min_val=255,max_val=0, sum=0, count=0;
    bool valid=true;
    unsigned long timeLastUpdate, timeLastRead;

    String _name;
    Adafruit_VL6180X sensor;
    bool _connected;
    Threads::Mutex _lock;
    int thread_id = -1;

    float _stable_value=-1;
    bool _is_stable;
    float c0=0,c1=1;
    
};

} // distance_sensor
