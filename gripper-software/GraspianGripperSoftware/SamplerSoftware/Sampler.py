
from os import sep
from threading import Thread, Lock
import time
from tkinter.constants import NO
import numpy as np
from numpy.core.fromnumeric import shape

import SensorInterface
import Mark10Controller

STATE_RESET       = "RESET"
STATE_SAMPLING    = "SAMPLING"
STATE_PAUSED      = "PAUSED"
STATE_DONE        = "DONE"

TYPE_SINGLE  = 1
TYPE_TIMED   = 2
TYPE_AUTO    = 3
TYPE_MONITOR = 4

SIGNAL_SEPRATOR = ", "

AUTO_TRIGGER_CONTROL_RUNNING = 1
AUTO_TRIGGER_CONTROL_FLAG1 = 2
AUTO_TRIGGER_FORCE_GTX = 3
AUTO_TRIGGER_ANY_SIGNAL_GTX = 4
AUTO_TRIGGER_ALL_SIGNAL_GTX = 5

#              Header column name
INCL_OPT_TIME     = "Time"
INCL_OPT_X        = "X"
INCL_OPT_FORCE    = "Force"
INCL_OPT_POS      = "Pos"
INCL_OPT_SETPOINT = "Set point"
INCL_OPT_SENSOR   = "Sensor"

class Sampler(Thread):
    def __init__(self, sensor_interface:SensorInterface.SensorInterface, mark_interface:Mark10Controller.Controller=None):
        super().__init__()
        self.sensor = sensor_interface
        self.mark  = mark_interface
        self.callbacks = []

        self.sensor.subscribe(self.sensor_callback)
        if(self.mark!=None):
            self.mark.subscribe(self._callback)

        self.include_options = {
            INCL_OPT_TIME:False,
            INCL_OPT_X:False,
            INCL_OPT_POS:False,
            INCL_OPT_FORCE:False,
            INCL_OPT_SETPOINT:False,
            INCL_OPT_SENSOR:False
        }
        self.reset_sampling_options()

        self._last_sample_id = [0, 0]

        self.first_sample_time = -1
        self._sampling_header = []
        self._sampling_data = np.empty((0,0))
        self.sampling_max_lines = -1
        self._sampling_lock = Lock()

        self.state = STATE_RESET
        self.sampling_type = TYPE_SINGLE
        # self.auto_sampling_started = False           
        self.start()

    def subscribe(self, cb):
        if(cb not in self.callbacks):
            self.callbacks.append(cb)

    def sensor_callback(self, event):
        if(event == SensorInterface.CB_CONNECTION_CLOSED):
            self.pause_sampling()
        elif(event == SensorInterface.CB_SIGNAL_SELECTION_CHANGED or event == SensorInterface.CB_SIGNALS_CHANGED):
            if(self.include_options[INCL_OPT_SENSOR]): self.include_options_changed()
        # self._callback()

    def _callback(self):
        for cb in self.callbacks: cb()

    def _check_trigger(self, trigger:str):
        negate_trigger = self.auto_samp_opt[trigger]<0
        trigger_enum   = abs(self.auto_samp_opt[trigger])
        if(  self.auto_samp_opt[trigger] == AUTO_TRIGGER_CONTROL_RUNNING): b = self.mark.control_is_active()
        elif(self.auto_samp_opt[trigger] == AUTO_TRIGGER_CONTROL_FLAG1):   b = self.mark.sampling_suggested()
        elif(self.auto_samp_opt[trigger] == AUTO_TRIGGER_FORCE_GTX):       b = self.mark.current_force() > self.auto_samp_opt["x"]
        elif(self.auto_samp_opt[trigger] == AUTO_TRIGGER_ANY_SIGNAL_GTX):  b = np.any(self.sensor.current_signals() > self.auto_samp_opt["x"])
        elif(self.auto_samp_opt[trigger] == AUTO_TRIGGER_ALL_SIGNAL_GTX):  b = np.all(self.sensor.current_signals() > self.auto_samp_opt["x"])
        else:
            print("Warning: Unknown trigger!")
            return False

        if(negate_trigger): return not b
        else:               return b

        # if(  self.auto_samp_opt[trigger] == AUTO_TRIGGER_CONTROL_RUNNING): return self.mark.control_is_active()
        # elif(self.auto_samp_opt[trigger] == AUTO_TRIGGER_CONTROL_FLAG1):   return self.mark.sampling_suggested()
        # elif(self.auto_samp_opt[trigger] == AUTO_TRIGGER_FORCE_GTX):       return self.mark.current_force() > self.auto_samp_opt["x"]
        # elif(self.auto_samp_opt[trigger] == AUTO_TRIGGER_ANY_SIGNAL_GTX):  return np.any(self.sensor.current_signals() > self.auto_samp_opt["x"])
        # elif(self.auto_samp_opt[trigger] == AUTO_TRIGGER_ALL_SIGNAL_GTX):  return np.all(self.sensor.current_signals() > self.auto_samp_opt["x"])
        # else:
        #     print("Warning: Unknown trigger!")
        #     return False

    def _auto_start_trigger(self):
        return self.auto_samp_opt["start"]!=None and self._check_trigger("start")

    def _auto_pause_trigger(self):
        return self.auto_samp_opt["pause"]!=None and self._check_trigger("pause")==False

    def _auto_stop_trigger(self):
        return self.auto_samp_opt["stop"]!=None and self._check_trigger("stop")==False

    def run(self):

        self._running=True
        while(self._running):
            try:
                if(self.state != STATE_SAMPLING):
                    time.sleep(0.01)
                    continue

                if(self.sampling_type==TYPE_AUTO):
                    self.sample_start_retained = self.sample_start_retained and self._auto_start_trigger()==False
                    if(self.sample_start_retained==False):
                        if(self._auto_stop_trigger()): # and self._sampling_data.shape[0] > 0):
                            print("STOP")
                            self.stop_sampling()
                            continue
                        elif(self._auto_pause_trigger()):
                            print("PAUSE")
                            self.sample_start_retained = True

                if(not self.sample_start_retained):

                    if(self.first_sample_time == -1):
                        self.first_sample_time = time.time()
                        self.next_sample_time=-1

                    cur_time = time.time() - self.first_sample_time
                    if(self.include_options[INCL_OPT_SENSOR] or self.include_options[INCL_OPT_FORCE] or self.include_options[INCL_OPT_POS]):
                        sample_is_new = self.sensor.current_signal_id() != self._last_sample_id[0] or self.mark.current_signal_id() != self._last_sample_id[1]
                        sample_is_valid = self.sensor.signals_are_valid() or self.include_options[INCL_OPT_SENSOR]==False
                    else:
                        sample_is_new = True
                        sample_is_valid = True
                            
                    if(cur_time >= self.next_sample_time and sample_is_new and sample_is_valid):
                        with self._sampling_lock:
                            if(self.sampling_max_lines>0 and self._sampling_data.shape[0] >= self.sampling_max_lines):
                                self._sampling_data = np.roll(self._sampling_data, -1, axis=0)
                                self._sampling_data = self._sampling_data[0:self.sampling_max_lines,:]
                                self._sampling_data[-1,:] = self._make_sample()
                            else:
                                self._sampling_data = np.vstack((self._sampling_data, self._make_sample()))

                            self._last_sample_id = [self.sensor.current_signal_id(), self.mark.current_signal_id()]
    
                        self.next_sample_time = cur_time+self.sampling_interval

                        if(self._sampling_data.shape[0] == self.sampling_num_samples):
                            self.stop_sampling()
                
                time.sleep(0.001)

            except KeyboardInterrupt:
                break
            except Exception as e: 
                print(e)

        print("Sampling loop ended!")

    def _update_header(self):
        header = []
        for key,value in self.include_options.items():
            if(value and key!=INCL_OPT_SENSOR): header.append(key)

        if(self.include_options[INCL_OPT_SENSOR]): 
            for i in range(1, self.sensor.number_of_signals()+1):
                header.append("T"+str(i))

        self._sampling_header = header

    def _make_sample(self, x=0) -> np.ndarray:
        sample = np.zeros((len(self._sampling_header)))
        i=0
        if(self.include_options[INCL_OPT_TIME]):     sample[i] = time.time()-self.first_sample_time; i+=1
        if(self.include_options[INCL_OPT_X]):        sample[i] = x;                                  i+=1
        if(self.include_options[INCL_OPT_POS]):      sample[i] = self.mark.current_position();       i+=1
        if(self.include_options[INCL_OPT_FORCE]):    sample[i] = self.mark.current_force();          i+=1
        if(self.include_options[INCL_OPT_SETPOINT]): sample[i] = self.mark.current_set_force();      i+=1
        
        if(self.include_options[INCL_OPT_SENSOR] and self.sensor.signals_are_valid()): sample[i:] = self.sensor.current_signals()
        return sample

    def sensor_available(self):
        return self.sensor.isConnected()
        # return self.sensor.number_of_signals()>0

    def force_available(self):
        return self.mark!=None and self.mark.is_ok()

    def position_available(self):
        return self.mark!=None and self.mark.is_ok()

    def sampling_is_possible(self):
        for _,value in self.available_include_options().items():
            if(value): return True
        return False

    def include_options_changed(self):
        self._update_header()
        self._callback()

    def set_include_option(self, option, b):
        if(self.state != STATE_RESET): return
        self.include_options[option] = b
        self.include_options_changed()

    def set_include_time(self, b):
        self.set_include_option(INCL_OPT_TIME, b)

    def set_include_sensor(self, b):
        self.set_include_option(INCL_OPT_SENSOR, b)

    def set_include_force(self, b):
        self.set_include_option(INCL_OPT_FORCE, b)

    def set_include_position(self, b):
        self.set_include_option(INCL_OPT_POS, b)

    def set_include_setpoint(self, b):
        self.set_include_option(INCL_OPT_SETPOINT, b)

    def available_include_options(self):
        avail_include_options = self.include_options.copy()
        avail_include_options[INCL_OPT_FORCE]    = avail_include_options[INCL_OPT_FORCE]    and self.force_available()
        avail_include_options[INCL_OPT_POS]      = avail_include_options[INCL_OPT_POS]      and self.position_available()
        avail_include_options[INCL_OPT_SETPOINT] = avail_include_options[INCL_OPT_SETPOINT] and self.force_available()
        avail_include_options[INCL_OPT_SENSOR]   = avail_include_options[INCL_OPT_SENSOR]   and self.sensor_available()
        return avail_include_options

    def get_include_options(self):
        return self.include_options.copy()

    def get_include_option(self, option):
        return self.include_options[option]

    def sample_size(self):
        return len(self._sampling_header)

    def reset(self):
        self.state=STATE_RESET
        self.sensor.unlock_signals()
        self._callback()

    def reset_sampling_options(self):
        self.sampling_interval = 0
        self.sampling_num_samples = -1
        self.sampling_max_lines = -1
        self.sample_start_retained = False
        self.auto_samp_opt:dict = {}
        self.include_options[INCL_OPT_X]=False
        self.include_options = self.available_include_options()
        print("reset_sampling_options - " + str(self.include_options))

    def allow_single_sample(self):
        return (self.state==STATE_RESET or self.state==STATE_PAUSED) and self.sampling_is_possible()

    def allow_sampling(self):
        return self.state==STATE_RESET and self.sampling_is_possible()

    def single_sample(self, x=None):
        if(self.allow_single_sample()==False): print("Warning: single sampling not possible"); return

        if(self.state==STATE_RESET):
            self.sampling_type = TYPE_SINGLE
            self.reset_sampling_options()
            self.include_options[INCL_OPT_X] = (x!=None)
            if(self.include_options[INCL_OPT_SENSOR]): self.sensor.lock_signals()
            self.first_sample_time = time.time()
            self._update_header()
            self._sampling_data = np.empty((0,len(self._sampling_header)))
            self.state=STATE_PAUSED
        elif(self.include_options[INCL_OPT_X] and x==None):
            return

        self._sampling_data = np.vstack((self._sampling_data, self._make_sample(x=x)))
        self._callback()

    def start_monitoring(self, interval, buffer_size):
        self.reset_sampling_options()
        self.sampling_type = TYPE_MONITOR
        self.sampling_interval = interval
        self.sampling_max_lines = buffer_size
        self._start_sampling()

    def start_auto_sampling(self, interval, auto_option):
        self.reset_sampling_options()
        self.sampling_type = TYPE_AUTO
        self.sampling_interval = interval
        self.sample_start_retained = True
        self.auto_samp_opt = auto_option
        self._start_sampling()

    def start_timed_sampling(self, interval, samples=-1):
        self.reset_sampling_options()
        self.sampling_type = TYPE_TIMED
        self.sampling_interval = interval
        self.sampling_num_samples = samples
        self._start_sampling()

    def _start_sampling(self):
        if(self.allow_sampling()==False): print("Warning: sampling not possible"); return
        print("_start_sampling - " + str(self.include_options) + " n_sgnals: " + str(self.sensor.number_of_signals()))
        self._update_header()
        self.sensor.lock_signals()
        self._last_sample_id = [self.sensor.current_signal_id(), self.mark.current_signal_id()]

        self.first_sample_time = -1
        self.next_sample_time = -1

        self._sampling_data = np.empty((0,len(self._sampling_header)), float)

        self.state = STATE_SAMPLING
        self._callback()

    def current_sampling_type(self):
        return self.sampling_type

    def pause_sampling(self):
        if(self.state==STATE_SAMPLING):
            self.state = STATE_PAUSED
            self._callback()

    def continue_sampling(self):
        if(self.state==STATE_PAUSED):
            self.state = STATE_SAMPLING
            self._callback()

    def stop_sampling(self):
        if(self.state==STATE_SAMPLING or self.state==STATE_PAUSED):
            self.state = STATE_DONE
            self.sensor.unlock_signals()
            self._callback()

    def allow_sampling(self):
        return self.state==STATE_RESET and self.sampling_is_possible()

    def sampling_stopped(self):
        return self.state!=STATE_SAMPLING

    def sampling_header(self) -> list:
        with self._sampling_lock:
            return self._sampling_header

    def sampling_data(self) -> np.ndarray:
        with self._sampling_lock:
            return self._sampling_data.copy()

    def sampling_columns(self) -> np.ndarray:
        with self._sampling_lock:
            return self._sampling_data.shape[1]

    def import_data(self, fname:str):
        if(self.state==STATE_SAMPLING): return
        with open(fname) as f:
            firstline = f.readlines()[0].rstrip()
            self._sampling_header = [s.strip() for s in firstline.split(',')]

        self._sampling_data = np.genfromtxt(fname, skip_header=1, delimiter=',')
        self.state = STATE_PAUSED
        self._callback()

    def shutdown(self):
        print("Sampler shutdown called")
        self._running=False

    