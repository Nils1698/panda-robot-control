from numpy.core.numeric import Inf
import serial
import time
import numpy as np
from matplotlib import pyplot as plt
from threading import Lock, Thread

#
# Tests
#  - ports stand/gauge (test response on 'x')
#  - PC Control
#  - travel unit
#  - force unit
#  - data mode on/off
#  - check heartbeats
#
#

BAUD = 115200

PORT_STAND = '/dev/ttyUSB0'
PORT_GAUGE = '/dev/ttyUSB1'

TRAVEL_UNITS = ["mm","in"]
FORCE_UNITS = ["N","lbF","oxF","kgF","gF"]

TRAVEL_UNIT = "mm"
FORCE_UNIT = "N"
SPEED_UNIT = TRAVEL_UNIT+"/min"

MIN_SPD = 0.5
MAX_SPD = 1100
POS_RES = 0.05 # mm

APPROACH_SPEED = 200 # mm/min
DETECTION_FORCE = 0.020 # N
MOVE_INTERUPT_FORCE = 20 #N
#MIN_FORCE_ERROR = 0.020 # min error before initating movement
MIN_FORCE_ERROR = 0.0

FORCE_MIN_LIMIT = 0.05 # N
FORCE_MAX_LIMIT = 20 # N
SAFETY_FORCE_LIMIT = 23 # N

STATUS_NOT_CONNECTED = 0
STATUS_OK = 1
STATUS_NOT_ZERO = 2
STATUS_DISCONNECTING = 3
STATUS_FAILED_CONNECTION = -1
STATUS_NO_RESPONSE = -2
STATUS_NO_CONTROL = 3
STATUS_WRONG_UNIT = -4
STATUS_WRONG_RESPONSE = -5
STATUS_READING_FAILED = -6

CONN_NOT_SET = 0
CONN_THROUGH_STAND = 1
CONN_INDIVIDUAL = 2

IF_REQ_POS       = b'x\n'
IF_REQ_FORCE_POS = b'n\n'
IF_REQ_SPD       = b'a\n'
IF_SET_SPD       = b'o\n'
IF_CTRL_UP       = b'u\n'
IF_CTRL_DOWN     = b'd\n'
IF_CTRL_STOP     = b's\n'
IF_ZERO_POS      = b'z\n'
IF_ZERO_FORCE    = b'R\n'
IF_UPPER_LIM     = b'h\n'
IF_LOWER_LIM     = b'g\n'
IF_REQ_UPPER_LIM = b'v\n'
IF_REQ_LOWER_LIM = b'w\n'
IF_REQ_STATUS    = b'p\n'

IF_FORCE_OVER_MSG = "OVERRANGE"

MOVE_STATUS_UP = 1
MOVE_STATUS_DOWN = -1
MOVE_STATUS_STOP = 0

def open_port(port_name):
    try:
        serial_port = serial.Serial(port_name,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE,
                parity= serial.PARITY_NONE,timeout=0.5)

        return serial_port

    except OSError as e:
        print("Failed to open port '{}'".format(port_name))
        return None

class Mark10Stand():
    def __init__(self, port_name:str):
        self.port_name = port_name
        self.port_lock = Lock()

        self.port = open_port(self.port_name)
        if(self.port != None):
            self.port.timeout = 0.05
            self.port.inter_byte_timeout = 0.05
            # TODO what if port didn't open!

    def send(self, code):
        with self.port_lock:
            self.port.write(code)

    def read_response(self) -> str:
        try:
            with self.port_lock:
                return self.port.readline().decode().replace("\r","").replace("\n","")
        except serial.SerialException:
            print("Stand - Lost conenction!")
            self._status=STATUS_FAILED_CONNECTION

    def request(self, code):
        print(f"send: {code}")
        with self.port_lock:
            self.port.write(code)
            return self.port.readline()

    def clear_input_buffer(self):
        try:
            with self.port_lock:
                while(1):
                    resp = self.port.readline()
                    if(resp==b''): break
        except serial.SerialException:
            print("Stand - Lost conenction!")
            self._status=STATUS_FAILED_CONNECTION

    def disconnect(self):
        if(self.port):
            self.send(b's')
            self.port.close()
            self.port = None

class Mark10Interface():
    def __init__(self, port_name:str=None):
        self._travel_status = MOVE_STATUS_STOP

        self.connection_type = CONN_NOT_SET
        self.current_spd=0

        self.stand=None
        self.callbacks=[]
        self.move_cmd_count = 0

        self.value_lock = Lock()

        if(port_name!=None):
            self.connect_stand(port_name) # note: sets connection type
        else:
            self._status = STATUS_NOT_CONNECTED

    def connect_stand(self, port_name):
        self.stand = Mark10Stand(port_name)
        self._status = self._initial_stand_check()
        self.stand.clear_input_buffer()
        self.set_upper_limit(500)
        self.set_lower_limit(-500)
        self._callback()

    def connect_gauge(self, port_name):
        print("connect_gauge - not implemented")
        # self.gauge = Mark10Gauge(port_name)
        # self.connection_type = CONN_INDIVIDUAL

    def subscribe(self, cb):
        if(cb not in self.callbacks):
            self.callbacks.append(cb)

    def clear_subscriptions(self):
        self.callbacks=[]

    def _callback(self):
        for cb in self.callbacks: cb()

    def error_string(self):
        if(self._status==STATUS_NO_RESPONSE):
            return "no response"
        if(self._status==STATUS_FAILED_CONNECTION):
            return "connection failed"
        if(self._status==STATUS_NO_CONTROL):
            return "no control"
        if(self._status==STATUS_WRONG_UNIT):
            return "wrong unit"
        if(self._status==STATUS_WRONG_RESPONSE):
            return "wrong repsonse"
        if(self._status==STATUS_READING_FAILED):
            return "reading error"

        return ""

    def _initial_stand_check(self):
        response = self.stand.request(IF_REQ_POS)
        print(f"resp: '{response}'")

        # Check response
        if(response==b''):
            print("init check - no response")
            return STATUS_NO_RESPONSE

        # Check travel units
        is_stand = False
        for u in TRAVEL_UNITS:
            str = response.decode()
            is_stand = is_stand or (u in str)
        if(not is_stand):
            print("init check - wrong response")
            return STATUS_WRONG_RESPONSE

        # Check speed control
        response = self.stand.request(IF_REQ_SPD)
        print(f"resp: '{response}'")
        if(response==b''):
            print("init check - no control")
            return STATUS_NO_CONTROL

        # Check gauge connection
        response = self.stand.request(IF_REQ_FORCE_POS)
        print(f"resp: '{response}'")
        if(response==b''):
            self.connection_type = CONN_INDIVIDUAL
            print("Gauge not connected throug stand")
        else:
            self.connection_type = CONN_THROUGH_STAND
            self.stand.clear_input_buffer() # ignore position response

        return STATUS_OK

    def is_ok(self):
        return self._status==STATUS_OK

    def is_stopped(self):
        return self._travel_status==MOVE_STATUS_STOP

    def zero_position(self):
        self.stand.send(IF_ZERO_POS)
        with self.value_lock:
            self._current_position=0

    def zero_force(self):
        if(self.connection_type == CONN_THROUGH_STAND):
            self.stand.send(IF_ZERO_FORCE)

        elif(self.connection_type == CONN_INDIVIDUAL):
            # TODO
            pass

    def _convert_position(self, msg:str):
        try:
            return float(msg.replace(TRAVEL_UNIT, ""))
        except ValueError:
            self._reading_failed()
            print("Position conversion error: "+msg)
            return 0

    def _convert_force(self, msg:str):
        try:
            return float(msg.replace(FORCE_UNIT, ""))
        except ValueError:
            self._reading_failed()
            print("Force conversion error: "+msg)
            return 0

    def read_position(self):
        response = self.stand.request(IF_REQ_POS)
        return self._convert_position(response)

    def read_force_and_position(self):
        print("read force pos ..")
        force = 0
        pos = 0

        if(self.connection_type == CONN_THROUGH_STAND):
            self.stand.send(IF_REQ_FORCE_POS)
            response = self.stand.read_response()
            print(f"force resp: '{response}'")
            force = self._convert_force(response)
            response = self.stand.read_response()
            print(f"pos resp: '{response}'")
            pos = self._convert_position(response)

        elif(self.connection_type == CONN_INDIVIDUAL):
            pos = self.read_position()
            # TODO

        return (force,pos)

    def _reading_failed(self):
        self._stop_control = True
        self._status = STATUS_READING_FAILED
        print("READING FAILED")
        self._callback()
        return

    def set_speed(self, mm_pr_min):
        if(self._status!=STATUS_OK): return
        msg = f"e{mm_pr_min:06.1f}" # format as 'eXXXX.X'
        self.stand.send(msg.encode()) # set speed
        self.stand.send(IF_SET_SPD)     # use set speed
        self.current_spd = max(min(round(mm_pr_min/MIN_SPD)*MIN_SPD, MAX_SPD), -MAX_SPD)

    def set_upper_limit(self, limit):
        if(self._status!=STATUS_OK): return
        if(limit>=0):
            msg = f"h{limit:06.2f}" # format as 'hXXX.XX'
        else:
            msg = f"h{limit:07.2f}" # format as 'h-XXX.XX'

        self.stand.send(msg.encode()) # set speed

    def set_lower_limit(self, limit):
        if(self._status!=STATUS_OK): return
        if(limit>=0):
            msg = f"g{limit:06.2f}" # format as 'gXXX.XX'
        else:
            msg = f"g{limit:07.2f}" # format as 'g-XXX.XX'

        self.stand.send(msg.encode()) # set speed

    def read_limits(self):
        if(self._status!=STATUS_OK): return
        self.stand.send(IF_REQ_LOWER_LIM)
        lo_lim = self._convert_position(self.stand.read_response())
        self.stand.send(IF_REQ_UPPER_LIM)        
        hi_lim = self._convert_position(self.stand.read_response())
        return (lo_lim,hi_lim)

    def read_status(self) -> list:
        self.stand.send(IF_REQ_STATUS)
        response = self.stand.read_response()
        return response.split(" ")

    def up(self):
        if(self._status!=STATUS_OK): return
        if(self._travel_status == MOVE_STATUS_STOP):
            self.stand.send(IF_CTRL_UP)
            self.move_cmd_count=0
        elif(self._travel_status == MOVE_STATUS_DOWN):
            self.stand.send(IF_CTRL_STOP)
            time.sleep(min(0.005*self.current_spd, 0.5))
            self.stand.send(IF_CTRL_UP)
            self.move_cmd_count=0
        else:
            self.move_cmd_count+=1
            if(self.move_cmd_count>10):
                self.stand.send(IF_CTRL_UP)
                self.move_cmd_count=0
        
        self._travel_status = MOVE_STATUS_UP

    def down(self):
        if(self._status!=STATUS_OK): return
        if(self._travel_status == MOVE_STATUS_STOP):
            self.stand.send(IF_CTRL_DOWN)
            self.move_cmd_count=0
        elif(self._travel_status == MOVE_STATUS_UP):
            self.stand.send(IF_CTRL_STOP)
            time.sleep(min(0.005*self.current_spd, 0.5))
            self.stand.send(IF_CTRL_DOWN)
            self.move_cmd_count=0
        else:
            self.move_cmd_count+=1
            if(self.move_cmd_count>10):
                self.stand.send(IF_CTRL_DOWN)
                self.move_cmd_count=0

        self._travel_status = MOVE_STATUS_DOWN

    def stop(self):
        if(self.stand == None): return
        self.stand.send(IF_CTRL_STOP)
        self._travel_status = MOVE_STATUS_STOP

    def command_velocity(self, vel):
        if(vel > MIN_SPD):
            self.set_speed(vel)
            self.up()
            vel = min(round(vel/0.5)*0.5, MAX_SPD)
        elif(vel < -MIN_SPD):
            self.set_speed(-vel)
            self.down()
            vel = max(round(vel/0.5)*0.5, -MAX_SPD)
        else:
            self.stop()
            vel = 0

        return vel

        # spd_mm_min = abs(vel)
        # if(vel>MIN_SPD):
        #     self.set_speed(spd_mm_min)
        #     self.up()
        # elif(vel<-MIN_SPD):
        #     self.set_speed(spd_mm_min)
        #     self.down()
        # else:
        #     self.stop()
        #     return 0

        # return self.current_spd

    def disconnect(self):
        if(self.connection_type == CONN_THROUGH_STAND):
            self.stand.disconnect()
            
        elif(self.connection_type == CONN_INDIVIDUAL):
            self.stand.disconnect()
            # TODO Disconnect gauge

        self._status=STATUS_NOT_CONNECTED
        self._callback()

def plot_log(log_name, indices):
    log = np.loadtxt(log_name)

    plt.figure()
    for i in indices:
        plt.plot(log[:,0],log[:,i])

    plt.grid()
    plt.show()

def clamp(value, abs_limit):
  if(value > abs_limit): return abs_limit
  if(value < -abs_limit): return -abs_limit
  return value

def moving_average(a, n):
    return np.convolve(a, np.ones((n,))/n, mode='same') 
    # ret = np.cumsum(a, dtype=float)
    # ret[n:] = ret[n:] - ret[:-n]
    # return ret[n - 1:] / n

class Controller(Mark10Interface, Thread):
    CONTROL_INACTIVE = False
    CONTROL_ACTIVE = True

    def __init__(self):
        # super(Controller, self).__init__()
        Mark10Interface.__init__(self)
        Thread.__init__(self)
        self._control_status=self.CONTROL_INACTIVE # TODO Lock
        self._control_function = None
        self._shutdown=False
        self._stop_control = False
        self._control_dict = {}
        self._t_init = 0
        self._t_last = 0
        self._done_count = 0
        self._active_hold = True
        
        self._current_set_force=0
        self.detection=False

        # LOG (debug)
        self.log_N = 10000
        self.log = np.zeros((self.log_N,6))
        self.log_i=0
        self.log_name = ""

        self._current_position = 0
        self._current_force = 0
        self._sampling_suggested=False
        self.callbacks = []

        self._current_signal_id:int = 0

        fs = 50 # Hz
        self._T = 1/fs

        self._update_force_and_pos()
        self.start()

    def sampling_suggested(self):
        with self.value_lock:
            return self._sampling_suggested

    def control_is_ready(self):
        return self.is_ok() and not self.control_is_active() and self.is_stopped()


    def current_force(self):
        with self.value_lock:
            return self._current_force

    def current_set_force(self):
        with self.value_lock:
            return self._current_set_force

    def current_position(self):
        with self.value_lock:
            return self._current_position        

    def control_is_active(self):
        return (self._control_status != self.CONTROL_INACTIVE)
        
    def run(self):
        
        while(not self._shutdown):
            if(not self.is_ok()):
                time.sleep(0.01)
                continue

            self.cur_t = time.time()
            if(self.cur_t >= self._t_last+self._T):
                self._t_last = self.cur_t

                self._update_force_and_pos()

                if(self._control_status != self.CONTROL_INACTIVE):
                    if(self._fail_check() or self._control_function(self._control_dict)):
                        print("Finish control!")
                        self._control_status=self.CONTROL_INACTIVE
                        self._control_finished()

            time.sleep(0.005)

    def _fail_check(self):
        return False

    def _update_force_and_pos(self):
        with self.value_lock:
            self._current_force,self._current_position = self.read_force_and_position()
            self._current_signal_id += 1
        self.detection = self._current_force > 0.005

    def current_signal_id(self):
        with self.value_lock:
            return self._current_signal_id

    def _begin_control(self):
        if(not self.control_is_ready()):
            print("Controller: control not ready")
            return False
        self.log_i=0
        self._t_init = time.time()
        self._t_last = self._t_init
        self._done_count = 0
        self._control_status=self.CONTROL_ACTIVE
        self._stop_control = False
        return True

    def stop_control(self):
        self._stop_control = True
        while(self._control_status != self.CONTROL_INACTIVE):
            time.sleep(0.1)

        with self.value_lock:
            self._sampling_suggested=False

        self._callback()
        print("control stopped") # debug

    def _control_finished(self):
        print("control done")
        self._callback()
        if(self.log_name != ""): 
            np.savetxt(self.log_name,self.log[0:self.log_i,:])

    def go_up(self, speed=-1):
        if(speed>0):
            self.set_speed(speed) # mm/min
        self._begin_move_control()
        self.up()

    def go_down(self, speed=-1):
        if(speed>0):
            self.set_speed(speed) # mm/min
        self._begin_move_control()
        self.down()

    def go_stop(self):
        self.stop_control()
        super().stop()

    def _begin_move_control(self):
        self._control_function = self._move_control
        return self._begin_control()

    def _move_control(self, _):
        if(self._status!=STATUS_OK or self._current_force>MOVE_INTERUPT_FORCE):
            self.stop()
            return True

        status = self.read_status()
        if( ('D' not in status and 'UL' in status) or ('U' not in status and 'DL' in status)): # at limit and not moving in opposite direction
            self.stop()
            return True

        return self._stop_control

    def goto(self, target_pos, relative=False, log=""):
        if(relative):
            target_pos += self._current_position

        self.log_name = log

        self._init_pos_control(target_pos)
        self._control_function = self._pos_control
        return self._begin_control()

    def _init_pos_control(self, target_pos):
        target_pos = round(target_pos/0.02)*0.02 # set point must be within (+/- 0.02)!
        lo,hi = self.read_limits()
        target_pos = max(target_pos, lo)
        target_pos = min(target_pos, hi)

        self._control_dict["pos_target"] = target_pos
        self._control_dict["vel"] = 0


    def _pos_control(self, dict):
        # Should run at 50 Hz
        KP = 80
        KD = 0.7

        if(self._stop_control or self._status!=STATUS_OK):
            self.stop()
            return True

        pos = self._current_position
        err = dict["pos_target"]-pos
        err_d = dict["vel"]
        des_vel = KP*err + KD*err_d

        if(abs(err) < 1e-4):
            self._done_count += 2
        elif(abs(err) <= 0.02):
            self._done_count += 0.2

        if(self._done_count>=50):
            self.stop()
            return True

        dict["vel"] = self.command_velocity(des_vel)

        # Logging (debug)
        if(self.log_name!="" and self.log_i<self.log_N):
            self.log[self.log_i,0] = self.cur_t-self._t_init
            self.log[self.log_i,1] = pos
            self.log[self.log_i,2] = err
            self.log[self.log_i,3] = des_vel
            self.log_i+=1

        return False

    def velocity_control_set_target(self, des_vel):
        self._control_dict["velocity_target"] = des_vel

        print(self._control_function)
        print(self._control_status)
        if(self._control_status!=self.CONTROL_ACTIVE):
            print("Init vel control!")
            self._control_function = self._velocity_control
            self._init_velocity_control(des_vel)
            self._begin_control()
        elif(des_vel==0):
            self._stop_control()

    def _init_velocity_control(self, des_vel):
        self._control_dict["velocity_target"] = des_vel
        self._control_dict["target_last"] = des_vel
        self._control_dict["cmd_vel_last"] = 0
        pass

    def _velocity_control(self, dict):
        print("Vel control running")

        if(self._stop_control or self._status!=STATUS_OK or self._current_force>FORCE_MAX_LIMIT):
            self.stop()
            print("STOP CONTROL")
            return True

        # Should run at 50 Hz
        # A1 = -0.222
        # B0 = 0.611
        # B1 = 0.611

        # Low Pass Filter
        # cmd_vel = A1*dict["cmd_vel_last"] + B0*dict["velocity_target"] + B1*dict["target_last"]
        # self.command_velocity(cmd_vel)
        # dict["cmd_vel_last"] = cmd_vel
        # dict["target_last"] = dict["velocity_target"]

        self.command_velocity(dict["velocity_target"])

        return False

    def approach(self):
        if(self._current_force>0.010):
            print("Force not zero!")
            return

        self._init_approach_control()
        self._control_function = self._approach_control
        return self._begin_control()


    def _init_approach_control(self):
        self._control_dict["approach_speed"]=APPROACH_SPEED
        self._control_dict["approach_force"]=DETECTION_FORCE


    def _approach_control(self, dict):
        # Should run at 50 Hz

        if(self._stop_control or self._status!=STATUS_OK):
            self.stop()
            return True

        self.set_speed(dict["approach_speed"])
        self.down()
        if(self._current_force>dict["approach_force"] or self._status!=STATUS_OK):
            self.stop()
            return True

        return False

    def release(self):
        if(self._current_force==0):
            print("Force already zero!")
            return

        self._init_release_control(speed=200, lift=0.1)
        self._control_function = self._release_control
        return self._begin_control()


    def _init_release_control(self, speed=100, period=-1, lift=2):
        self._control_dict["init_release_pos"] = self._current_position
        self._control_dict["release_max_lift"] = 25 # mm
        self._control_dict["release_lift"] = lift # mm
        self._control_dict["release_force"] = 0.020
        self._control_dict["release_pos"] = 1000
        self._control_dict["release_init_time"]=time.time()
        self._control_dict["release_period"]=period
        self._control_dict["release_speed"]=speed

    def _release_control(self, dict):
        if(self._stop_control or self._status!=STATUS_OK
            or self._current_position-dict["init_release_pos"]>dict["release_max_lift"]):
            self.stop()
            print("Release control finished")
            return True

        if(dict["release_pos"] == 1000):
            if(self._current_force <= dict["release_force"]):
                dict["release_init_time"]=time.time()
                dict["release_pos"] = self._current_position
        else:
            if(self._current_position-dict["release_pos"]>dict["release_lift"]):
                self.stop()
                if(dict["release_period"]<0): return True

            if(dict["release_period"]>=0):
                return time.time()-dict["release_init_time"] > dict["release_period"]

        self.set_speed(dict["release_speed"])
        print(f'release speed: {dict["release_speed"]}')
        self.up()

        return False

    def _init_release_control2(self, speed=100, period=-1, lift=0.1):
        self._control_dict["release_state"]=0
        self._init_release_control(speed,period,lift)
        self._init_force_control2(0, 0, False)

    def _release_control2(self, dict):
        if(self._stop_control or self._status!=STATUS_OK):
            self.stop()
            return True

        if(dict["release_state"]==0):
            if(self._force_control2(dict)):
                dict["release_state"]=1
                print("release control - speed 100")
        else:
            return self._release_control(dict)


    def _init_force_control2(self, target_force, hold_period, period_includes_risetime:bool):
        target_force = round(target_force/0.005)*0.005 # set point must be within (+/- 0.005)!
        self._control_dict["force_target"]=target_force
        self._current_set_force=target_force
        self._control_dict["vel"]=0
        self._control_dict["target_reached"]=False

        self._control_dict["stop_after"]=hold_period
        self._control_dict["fixed_period"]=period_includes_risetime
        if(period_includes_risetime==False):
            self._control_dict["init_time"]=Inf
        else:
            self._control_dict["init_time"]=time.time()

    def _force_control2(self, dict):
        # Should run at 50 Hz

        if(self._stop_control or self._status!=STATUS_OK):
            self.stop()
            return True

        # print(f"Period: {time.time()-dict['init_time']}")
        if(time.time()-dict["init_time"] >= dict["stop_after"]):
            print("Control2 done!")
            self.stop()
            with self.value_lock:
                self._current_set_force=0
                self._sampling_suggested=False
            return True

        if(self._current_force>SAFETY_FORCE_LIMIT):
            self.up()
            self.set_speed(200)
            print(f"ERROR: reached force safty limit ({SAFETY_FORCE_LIMIT}N)")
            time.sleep(1)
            return True

        force_err = dict["force_target"]-self._current_force

        KS = 0.803# N/mm  Spring constant
        des_delta_pos = - 1/KS * force_err # feed forward
        adjust_force = dict["target_reached"]==False or self._active_hold==True
        if(abs(des_delta_pos) >= POS_RES and adjust_force):
            # print(f"V: {abs(des_delta_pos):.3} >= {POS_RES}") debug
            KP = 80; KD = 0.7
            des_vel = KP*des_delta_pos + KD*dict["vel"]
            dict["vel"] = self.command_velocity(des_vel)
        else:
            self.stop()
            # print(f"X: {abs(des_delta_pos):.3} < {POS_RES}") # debug
            if(dict["target_reached"] == False):
                dict["target_reached"] = True
                print("target reached")
                with self.value_lock: self._sampling_suggested=True
                if(not dict["fixed_period"]):
                    dict["init_time"] = time.time()

    def apply_forces(self, target_forces:np.array, hold_periods:np.array, active_hold=True):
        if(target_forces.min()<0 or target_forces.max()>FORCE_MAX_LIMIT):
            print(f"Force control: force exceeds force limit of [0;20N]!")
            return

        self._active_hold = active_hold
        self._init_force_step_control(target_forces, hold_periods)
        self._control_function = self._force_step_control
        return self._begin_control()

    def _init_force_step_control(self, force_steps, time_steps, include_risetime=False):
        if(force_steps[-1]!=0):
            force_steps = np.append(force_steps, 0)
            time_steps  = np.append(time_steps, -1)
        self._control_dict["force_steps"]=force_steps
        self._control_dict["time_steps"]=time_steps
        self._control_dict["current_step"]=0
        self._control_dict["total_steps"]=force_steps.size
        self._control_dict["step_release"]= force_steps[0]==0
        self._control_dict["step_incl_rise"]=include_risetime

        self._init_force_control2(force_steps[0],time_steps[0],include_risetime)

    def _force_step_control(self, dict):
        if(self._stop_control or self._status!=STATUS_OK):
            self.stop()
            return True

        if(dict["step_release"]):
            done = self._release_control2(dict)
        else:
            done = self._force_control2(dict)

        if(done):
            print("Step finished")
            dict["current_step"] += 1
            i = dict["current_step"]

            if(i==dict["total_steps"]):
                print("Step control finished completly!") # debug
                self.stop()
                return True                    

            dict["step_release"] = dict["force_steps"][i]==0
            if(dict["step_release"]):
                print("START Release control!") # debug
                self._init_release_control2(period=dict["time_steps"][i])
            else:
                self._init_force_control2( dict["force_steps"][i], dict["time_steps"][i], dict["step_incl_rise"])
                
        return False

    def apply_force_cycles(self, target_force, hold_period, release_period, cycles:int, increment=0, active_hold=True):
        if(target_force <0 or target_force>FORCE_MAX_LIMIT):
            print(f"Force control: {target_force}N exceeds force limit of 20N!")
            return

        self._active_hold = active_hold


        self._init_force_cycle_control(target_force, increment, hold_period, release_period, cycles)
        self._control_function = self._force_cycle_control
        return self._begin_control()

    def _init_force_cycle_control(self, target_force, increment, hold_period, release_period, cycles):
        self._control_dict["cycle_force"]=target_force
        self._control_dict["cycle_increment"]=increment
        self._control_dict["cycle_period"]=hold_period
        self._control_dict["cycle_release_period"]=release_period
        self._control_dict["total_cycles"]=cycles
        self._control_dict["cycle_incl_rise"]=True
        self._control_dict["current_cycle"]=1
        self._control_dict["release_time"]=0
        self._control_dict["state"]=0

        self._init_force_control2(target_force, hold_period, self._control_dict["cycle_incl_rise"])
        
    def _force_cycle_control(self, dict):
        if(self._stop_control or self._status!=STATUS_OK):
            self.stop()
            return True

        if(dict["state"]==0): # Apply force
            if(self._force_control2(dict)):                
                dict["release_time"]=time.time()
                self._init_release_control2(lift=0.1)
                dict["state"]=1

        elif(dict["state"]==1): # release
            if(self._release_control2(dict)):
                self.stop()
                if(dict["current_cycle"] == dict["total_cycles"]): return True
                dict["state"]=2

        elif(dict["state"]==2): # wait
            if(time.time() - dict["release_time"] > dict["cycle_release_period"]):
                dict["current_cycle"]+=1
                dict["cycle_force"] += dict["cycle_increment"]
                print("Next cycle: "+str(dict["current_cycle"]))
                self._init_force_control2(dict["cycle_force"],dict["cycle_period"],dict["cycle_incl_rise"])
                dict["state"]=0

        return False 


    def test_compliance(self, max_force:float, max_squeeze:float, speed:float):
        if(max_force > FORCE_MAX_LIMIT):
            print(f"Compliance control: {max_force}N exceeds force limit of 20N!")

        self._control_dict = {
            "max_force":max_force,
            "max_squeeze":max_squeeze,
            "comp_speed":speed,
            "init_pos":self._current_position,
            "state":0
        }
        self._init_approach_control()
        self._control_function = self._compliance_control
        return self._begin_control()

    def _compliance_control(self,dict):
        
        if(self._stop_control or self._status!=STATUS_OK):
            self.stop()
            print("STOP CONTROL")
            with self.value_lock:
                self._sampling_suggested=False
            return True

        # Approach
        if(dict["state"]==0):
            # if(self._approach_control(dict)):
            if(True):
                self.zero_position()
                
                dict["detect_pos"]=self._current_position
                self.command_velocity(-dict["comp_speed"])
                print("detection!")
                dict["state"]=1
                with self.value_lock:
                    self._sampling_suggested=True


        # Compliance Control
        if(dict["state"]==1):
            if(self._current_force>=dict["max_force"] or (self._current_position-dict["detect_pos"]) <= -dict["max_squeeze"]):
                self.stop()
                print("complinace done")
                with self.value_lock:
                    self._sampling_suggested=False
                # self._init_pos_control(dict["detect_pos"]+5)
                # print("goto: "+str(dict["detect_pos"]))
                # with self.value_lock:
                #     self._sampling_suggested=False
                self._init_release_control(speed=dict["comp_speed"], lift=0.1)
                dict["state"]=2
                
        # Return
        if(dict["state"]==2):
            # return self._pos_control(dict)
            if(self._release_control(dict)):
                # with self.value_lock:
                #     self._sampling_suggested=False
                return True

        return False

    def start_dynamic_control(self,force,speed,period,cycles):
        self._init_dynamic_control(force,speed,period,cycles)
        self._control_function = self._dynamic_control
        return self._begin_control()

    def _init_dynamic_control(self,force,speed,period,cycles):
        self._control_dict["dyn_force"] = force
        self._control_dict["dyn_speed"] = speed
        self._control_dict["dyn_period"] = period
        self._control_dict["dyn_cycles"] = cycles
        self._control_dict["dyn_time_init"] = time.time()
        self._control_dict["state"] = 0
        self._control_dict["dyn_cur_cycle"] = 0

    def _dynamic_control(self,dict):
        if(self._stop_control or self._status!=STATUS_OK):
            self.stop()
            return True

        # Press
        if(dict["state"]==0):

            self.command_velocity(-dict["dyn_speed"])
            if(self._current_force >= dict["dyn_force"]):
                dict["state"]+=1
                self.command_velocity(dict["dyn_speed"])

        # Release
        if(dict["state"]==1):
            if(self._current_force <= 0.005):
                self.stop()
                dict["dyn_cur_cycle"] += 1
                dict["state"]+=1
                if(dict["dyn_cur_cycle"]==dict["dyn_cycles"]): return True
                
        # Wait
        if(dict["state"]==2):
            if(time.time() - dict["dyn_time_init"] > dict["dyn_cur_cycle"]*dict["dyn_period"]):
                dict["state"] = 0
            
        return False

    def disconnect(self):
        self.stop_control()
        self._status = STATUS_DISCONNECTING
        time.sleep(0.1) # wait for force/position readings to finish
        Mark10Interface.disconnect(self)

    def shutdown(self):
        self.clear_subscriptions()
        self.disconnect()
        self._shutdown=True
        while(self.is_alive()):
            time.sleep(0.1)
    