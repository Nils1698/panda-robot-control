from threading import Thread, Lock

import socket
import time
import struct

import numpy as np

# Configure wired network with ip 192.168.1.x (e.g. x=99)

ROBOT_IP = "192.168.1.100" # Must match ur robot "Setiings->System->Network->IP"
# ROBOT SUBNET MASK: 255.355.255.0
# ROBOT DEFAULT GATEWAY: 255.355.255.0
ROBOT_PORT = 30002         

CONNECTION_TIMEOUT = 3

PORT_NOT_OPEN = 0
PORT_OPEN = 1
PORT_CLOSING = 2

DEFAULT_LIN_SPEED = 0.1 # m/s
DEFAULT_LIN_ACC = 0.6 # m/s^2
DEFAULT_JOINT_SPEED = 0.8 # rad/s
DEFAULT_JOINT_ACC = 1.5 # rad/s^2

CBEVENT_ROBOT_CONNECTED = "robot connected"
CBEVENT_ROBOT_DISCONNECTED = "robot disconnected"
CBEVENT_ROBOT_STATUS_CHANGED = "status changed"
CBEVENT_ROBOT_PROTECTIVE_STOP = "protective stop"

SAFETY_Z_LIMIT = 0.03

class RobotInterface(Thread):

    def __init__(self, autoconnect=False):
        super().__init__()
        self.callbacks = []
        self.port_status = PORT_NOT_OPEN

        self.robot_socket:socket.socket = None
        
        self._lock = Lock()
        self._program_running = False
        self._is_powered_on = False

        self.lin_spd = DEFAULT_LIN_SPEED
        self.lin_acc = DEFAULT_LIN_ACC

        self.joint_spd = DEFAULT_JOINT_SPEED
        self.joint_acc = DEFAULT_JOINT_ACC

        self.last_movel_pos = [-0.227, -0.134, 0.457] # home pos
        self.last_movel_orient = [2.18,-2.26,0] # home orient

        if(autoconnect): self.connect()

        self._enabled=True

        self._running = True
        self.start()

    def subscribe(self, cb):
        if(cb not in self.callbacks):
            self.callbacks.append(cb)

    def _callback(self, event):
        for cb in self.callbacks:
            cb(event)

    def enable(self):
        with self._lock:
            self._enabled=True

    def disable(self):
        with self._lock:
            self._enabled=False

    def connect(self):
        if(self.isConnected()):
            self.disconnect()

        try:
            print("Robot connecting ..")
            self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.robot_socket.settimeout(CONNECTION_TIMEOUT)
            self.robot_socket.connect((ROBOT_IP, ROBOT_PORT))
            self.port_status = PORT_OPEN
            print("  .. connected!")
            self._callback(CBEVENT_ROBOT_CONNECTED)
            self.enable()
            return True

        except Exception as e:
            print(e)
            return False

    def checkResponse(self):
        try:
            data = self.robot_socket.recv(4096)
        except socket.timeout:
            print("Socket timed out in receive!")
            self.port_status=PORT_NOT_OPEN
            self._callback(CBEVENT_ROBOT_DISCONNECTED)
            return
        except ConnectionResetError:
            print("Robot set to local control")
            self.port_status=PORT_NOT_OPEN
            self._callback(CBEVENT_ROBOT_DISCONNECTED)
            return

        i = 0
        package_length = (struct.unpack('!i', data[0:4]))[0]
        package_type = (struct.unpack('!B', data[4:5]))[0]
        # print("PKG TYPE: " + str(package_type))
        if(package_type == 16):
            while i + 5 < package_length:
                message_length = (struct.unpack('!i', data[5 + i:9 + i]))[0]
                message_type = (struct.unpack('!B', data[9:10]))[0]
                #print("MSG TYPE: " + str(message_type))
                if message_type == 0:
                    sub_package = data[5:i + message_length]
                    is_program_running = struct.unpack('!?', sub_package[18:19])[0]
                    is_protective_stop = struct.unpack('!?', sub_package[17:18])[0]
                    is_robot_on = struct.unpack('!?', sub_package[15:16])[0]
                    # print('PWR ON/STOP/PROG RUN : ' + str(is_robot_on) + '/' + str(is_protective_stop) + '/' + str(is_program_running))
                    if(is_protective_stop):
                        with self._lock:
                            self._callback(CBEVENT_ROBOT_PROTECTIVE_STOP)

                    with self._lock:
                        self._program_running = is_program_running

                    if(self._is_powered_on != is_robot_on):
                        with self._lock: self._is_powered_on = is_robot_on
                        self._callback(CBEVENT_ROBOT_STATUS_CHANGED)
                   
                    time.sleep(0.1)
                    break
                i = message_length + i

        #return tupleis_robot_on, is_protective_stop, is_program_running

    def run(self):
        i=0 # debug
        print("Robot thread running")
        while self._running:
            if(self.port_status==PORT_CLOSING):
                if(self.robot_socket != None): self.robot_socket.close()
                self.robot_socket = None
                self.port_status = PORT_NOT_OPEN
                print("robot socket closed")
            
            while self.port_status==PORT_OPEN:
                time.sleep(0.001) # yield thread
                self.checkResponse()
                # [PORT_OPEN loop] #

            time.sleep(0.1) # yield thread
            # [_running loop] #

        print("Robot loop ended")

    def is_active(self):
        with self._lock:
            return self._program_running

    def is_enabled(self):
        with self._lock:
            return self._enabled

    def execute(self, cmd:str, blocking):
        if(self.is_enabled()==False): return

        with self._lock:
            self._program_running = True        

        try:
            self.robot_socket.send((cmd+"\n").encode('utf8'))
        except BrokenPipeError:
            return

        time.sleep(0.2)
        while(blocking and self.is_enabled() and self.is_active()):
            time.sleep(0.1)

        print("robot execution finished")

    def movel_rel(self, rel_pos, blocking=True):
        print(f"move_rel {rel_pos}")
        self.movel(np.array(self.last_movel_pos) + np.array(rel_pos), blocking=blocking)

    def movel(self, pos, orient=[], blocking=True):
        if(pos[2] < SAFETY_Z_LIMIT):
            print(f"Z limit violation! z = {pos[2]} < {SAFETY_Z_LIMIT}")
            self.disable()
            return

        if(orient==[]):
            orient = self.last_movel_orient
            
        self.last_movel_pos = pos
        self.last_movel_orient = orient
        print(f"movel: {pos}")
        self.execute(f"movel(p[{pos[0]},{pos[1]},{pos[2]},{orient[0]},{orient[1]},{orient[2]}], a={self.lin_acc}, v={self.lin_spd})", blocking)

    def movej(self, q, spd=-1, blocking=True):
        print(f"movej: {q}")
        if(spd<0): spd = self.joint_spd
        self.execute(f"movej([{q[0]},{q[1]},{q[2]},{q[3]},{q[4]},{q[5]}], a={self.joint_acc}, v={spd})", blocking)

    def speedl(self, x,y,z,rx=0,ry=0,rz=0,acc=-1, timeout=-1, blocking=True):
        if(acc<=0): acc=self.lin_acc

        timeout_str=""
        if(timeout>0):
            timeout_str = f",{timeout}"

        self.execute(f"speedl([{x},{y},{z},{rx},{ry},{rz}],{acc}"+timeout_str+")",blocking)

    def stopj(self, deacc=-1, blocking=True):
        if(deacc<=0): deacc=self.joint_acc
        self.execute(f"stopj({deacc})",blocking)

    def stopl(self, deacc=-1, blocking=True):
        if(deacc<=0): deacc=self.lin_acc
        self.execute(f"stopl({deacc})",blocking)

    def move_home(self,blocking=True):
        self.movej([0, -2.094, 1.57, -1.047, -1.57, 0], blocking=blocking)

    def disconnect(self):        
        if(self.isConnected()):
            print("Close robot socket .. ")
            self.port_status = PORT_CLOSING
            i=0
            while(self.port_status != PORT_NOT_OPEN):
                time.sleep(0.01) # wait for monitor loop to end
                if(i>1/0.01):
                    raise Exception("Timed out closing robot socket!")
                    return False
                i+=1
        else:
            return False

        return True

    def isConnected(self):
        return self.port_status==PORT_OPEN

    def isPoweredOn(self):
        with self._lock:
            return self._is_powered_on

    def isReady(self):
        return self.is_enabled() and self.isPoweredOn()

    def shutdown(self):
        self.disconnect()
        self._running = False


if(__name__=="__main__"):
    robot = RobotInterface()
    robot.connect()
    time.sleep(0.5)


    robot.speedl(-0.02, 0, 0, timeout=2,blocking=False)
    time.sleep(4)
    # robot.move_home()
    # print("Move ..")
    # robot.movel([-0.3, 0.074, 0.5])
    # robot.movel([-0.4, 0, 0.6])
    # print("Done moving")
    # robot.move_home()

    robot.stopl()
    time.sleep(1)
    robot.shutdown()