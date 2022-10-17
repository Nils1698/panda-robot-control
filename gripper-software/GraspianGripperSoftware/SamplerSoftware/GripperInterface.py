
from threading import Lock

from matplotlib import blocking_input
from SensorInterface import *
import time
import ast

RESPONSE_TIMEOUT = 2 #s

CBEVENT_GRIPPER_OBJECT_DROPPED = "CBObjectDropped"

class GripperInterface(SensorInterface):

    def __init__(self, port=None):
        super().__init__()
        self._lock = Lock()
        self._ack_received = False
        self._is_active = False
        self._enabled = True
        self._result = ""

        self.callbacks = []

        if(port != None): self.connect(port)

    def subscribe(self, cb):
        if(cb not in self.callbacks):
            self.callbacks.append(cb)
            super().subscribe(cb)

    def _callback(self, event):
        for cb in self.callbacks:
            cb(event)

    def connect(self, port_name): # override
        if( super().connect(port_name) ):
            self.execute("status", blocking=False)

            if(self._ack_received):
                self.enable()
            else:
                self.port_status = PORT_NOT_OPEN
                self._callback(CB_CONNECTION_CLOSED)

    def enable(self):
        with self._lock:
            self._enabled=True

    def disable(self):
        print("GRIPPER DISABLED")
        with self._lock:
            self._enabled=False

    def isEnabled(self):
        with self._lock:
            return self._enabled

    def is_active(self):
        with self._lock:
            return self._is_active

    def command_acknowledged(self):
        with self._lock:
            return self._ack_received

    def execute(self, cmd, blocking):
        if(self.isEnabled()==False): return

        with self._lock:
            self._ack_received=False
            self._is_active=True

        time_init = time.time()
        while(not self.command_acknowledged()):
            if(self.is_active()==False): return
            if(time.time()-time_init > RESPONSE_TIMEOUT):
                print("Sending command to gripper timed out!")
                return

            self.send(cmd+"\n")
            time.sleep(0.8)

        while(blocking and self.isEnabled() and self.is_active()):
            time.sleep(0.1)

    def slip_grip(self, slip_thres=4, init_force=1, return_when_no_slip=-1):
        self.execute(f"slide grip {slip_thres} {init_force} {return_when_no_slip}", blocking=True)
        try:
            return float(self._result)
        except ValueError:
            return 0
        # self.execute("grip slip b 4", blocking=False)

    def slip_demo(self, blocking=True):
        self.execute("grip slip a", blocking=blocking)

    def grip_demo(self):
        self.execute("grip demo", blocking=False)

    def await_motion(self):
        self.execute(f"await motion", blocking=True)

    def open(self, blocking=True):
        self.execute("open", blocking)

    def stop(self):
        self.execute("stop", blocking=False)

    def home(self, blocking=True):
        self.execute("home", blocking)

    def lights(self, on):
        if(on):
            self.execute("led on", blocking=False)
        else:
            self.execute("led off", blocking=False)

    def get_status(self):
        self.send("status")

    def goto(self, width, blocking=True):
        self.execute(f"goto {width}", blocking)

    def gorel(self, width, blocking=True):
        self.execute(f"gorel {width}", blocking)

    def detect(self, speed=50):
        self.execute(f"detect {speed}", blocking=True)
        return self._result

    def close(self, force):
        self.execute(f"close {force}", blocking=True)

    def release(self, force=0.1, speed=10, max_change=-1, blocking=True):
        self.execute(f"release {force} {speed} {max_change}", blocking=blocking)

    def characterize_object(self):
        self.execute("char", blocking=True)
        if(self._result != ""):
            try:
                return ast.literal_eval(self._result)
            except ValueError:
                pass
            
        return {'cnt':0, 'obj':'unknown'}

    def hold_object(self):
        self.execute(f"hold obj", blocking=False)

    def hold_force(self, force):
        self.execute(f"hold {force}", blocking=False)

    def grip(self, blocking=True):
        self.execute(f"grip char", blocking)

    def handle_incoming(self, line):
        line = line.replace("\n","")
        if(line.startswith("done")):
            with self._lock:
                self._result = line[5:]
                self._is_active = False
        elif(line=="ack"):
            with self._lock:
                self._ack_received = True
        elif(line=="obj:drop"):
            print("GripperInterface: OBJECT DROPPED!")
            self._callback(CBEVENT_GRIPPER_OBJECT_DROPPED)
        # elif(line.startswith("obj:")):
        #     self.gripped_object = line[4:]
        #     print(f"Object is: '{self.gripped_object}'")
        #     with self._lock:
        #         self._is_active = False
        # elif(line.startswith("cnt:")):
        #     self.obj_zpos = float(line[4:])
        #     print(f"obj z pos: {self.obj_zpos}")
        #     with self._lock:
        #         self._is_active = False
        else:
            super().handle_incoming(line)

if(__name__=="__main__"):
    gripper = GripperInterface("/dev/ttyUSB0")
    gripper.execute("open", blocking=False)

    # gripper.detect()
    # gripper.close(1)
    # char = gripper.characterize_object()
    # print(char["obj"])

    # gripper.execute("detect", blocking=True)
    # print(gripper._result)

    # gripper.execute("close 2", blocking=True)

    # gripper.execute("char", blocking=True)
    # print(gripper._result)
    # a = ast.literal_eval(gripper._result)
    # print(a["cnt"])

    gripper.shutdown()
