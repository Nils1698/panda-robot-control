
from shutil import ExecError
import time
from tkinter import RIDGE

from numpy import block
from RobotInterface import RobotInterface
from GripperInterface import GripperInterface
from GripperInterface import *

pick_pos_z = 0.07
pick_pos_x_near = -0.35
pick_pos_x_far = -0.60
pick_pos_y_left = -0.08
pick_pos_y_right = 0.10

pos_left_near = np.array([pick_pos_x_near, pick_pos_y_left, pick_pos_z])
pos_left_far = np.array([pick_pos_x_far, pick_pos_y_left, pick_pos_z])

pos_right_near = np.array([pick_pos_x_near, pick_pos_y_right, pick_pos_z])
pos_right_far = np.array([pick_pos_x_far, pick_pos_y_right, pick_pos_z])

pos_present = np.array([-0.56, 0, 0.30])
rot_present = np.array([2.356, 2.356, -2.356])

joints_pre_back = np.array([-31, -103, 127, -20, 55, 0],dtype=float)/180*np.pi
joints_pre_right = np.array([-46, -71, 95, -26, -45, 2],dtype=float)/180*np.pi

# joints_present = np.array([-12.5, -66.1, 69.6, -155.3, -79.7, -7.2],dtype=float)/180*np.pi
joints_present = np.array([-12.5, -56.0, 70.0, -194.0, -77.5, 0],dtype=float)/180*np.pi

pos_present2 = np.array([-0.45, 0.06, 0.30])
rot_present2 = np.array([4.71, 0, 0])

joints_grip_demo = joints_pre_back

SIDE_RIGHT = 0
SIDE_LEFT = 1
SIDE_BACK = 2

POS_NEAR = 0
POS_FAR = 1

NO_DEMO = 0
MAIN_DEMO = 1
GRIP_DEMO = 2
SLIP_DEMO = 3

CBEVENT_DEMO_ENDED = "demo ended"
CBEVENT_DEMO_STARTED = "demo started"

OBJ_UNSET = "unset"
OBJ_UNKNOWN = "unknown"
OBJ_NONE = "none"
OBJ_EMPTY_CAN = "EmptyCan"
OBJ_FULL_CAN = "FullCan"
OBJ_TOMATO = "Tomato"
OBJ_MANDARIN = "Mandarin"

class Demo:

    def __init__(self, robot:RobotInterface, gripper:GripperInterface):
        self.robot = robot
        self.gripper:GripperInterface = gripper
        self._abort_demo=False

        self.callbacks = []

        self.demo_run_state = 0
        self.run_i = 0
        self.current_side = SIDE_RIGHT
        self._demo_running = False
        self._demo_started = NO_DEMO
        self.gripped_object = OBJ_UNSET

        self.gripper.subscribe(self.gripper_callback)

    def subscribe(self, cb):
        if(cb not in self.callbacks):
            self.callbacks.append(cb)

    def _callback(self, event):
        for cb in self.callbacks:
            cb(event)

    def gripper_callback(self, event):
        if(event==CBEVENT_GRIPPER_OBJECT_DROPPED):
            print("RobotGUI: object dropped")
            self.abort(move_home=True, after_delay=1.5)

    def isRunning(self):
        return self._demo_running

    def demoRunning(self):
        if(self._demo_running):
            return self._demo_started
        else:
            return NO_DEMO

    def currentDemo(self):
        return self._demo_started

    def abort(self, move_home=False, after_delay=0):
        self._abort_demo=True
        self.robot.disable()
        self.gripper.disable()
        while(self.isRunning()):
            time.sleep(0.1)

        self.robot.enable()
        self.gripper.enable()

        print("demo stopped running")

        self.robot.stopj()
        self.gripper.stop()
        
        if(move_home):
            time.sleep(after_delay)
            self.home()

    def pick_poses(self, pick_pos, side):
        if(side==SIDE_LEFT):
            pick_rot = np.array([4.71, 0, 0])
            dive_pos = pick_pos + np.array([0, -0.050, 0])
        elif(side==SIDE_RIGHT):
            pick_rot = np.array([0.034, 2.187, -2.187])
            dive_pos = pick_pos + np.array([0, 0.050, 0])
        elif(side==SIDE_BACK):
            pick_rot = np.array([2.356, 2.356, -2.356])
            dive_pos = pick_pos + np.array([0.05, 0, 0])
        else:
            raise "unknown value in side"

        dive_rot = pick_rot
        pre_dive_pos = dive_pos + np.array([0, 0, 0.150])
        pre_dive_rot = dive_rot
        lift_pos = pick_pos + np.array([0, 0, 0.050])
        lift_rot = pick_rot
        return [(pre_dive_pos,pre_dive_rot),(dive_pos,dive_rot),(pick_pos,pick_rot),(lift_pos,lift_rot)]

    def prepare_back_grip(self):
        self.robot.movej(joints_pre_back)

    def prepare_right_grip(self):
        self.robot.movej(joints_pre_right)

    def pick_from_side(self, pick_pos, side=SIDE_RIGHT):
        poses = self.pick_poses(pick_pos, side)
        pre_dive_pose = poses[0]
        dive_pose = poses[1]
        pick_pose = poses[2]

        self.robot.movel(pre_dive_pose[0],pre_dive_pose[1])
        self.robot.movel(dive_pose[0],dive_pose[1])
        self.robot.movel(pick_pose[0],pick_pose[1])

    def lift_from_side(self, pick_pos, side=SIDE_RIGHT):
        poses = self.pick_poses(pick_pos, side)
        pre_dive_pose = poses[0]
        lift_pose = poses[3]

        self.robot.movel(lift_pose[0],lift_pose[1])
        self.robot.movel(pre_dive_pose[0],pre_dive_pose[1])

    def place_from_side(self, pick_pos, side=SIDE_RIGHT):
        poses = self.pick_poses(pick_pos, side)
        pre_dive_pose = poses[0]
        pick_pose = poses[2]
        lift_pose = poses[3]
   
        self.robot.movel(pre_dive_pose[0],pre_dive_pose[1])
        self.robot.movel(lift_pose[0],lift_pose[1])
        self.robot.movel(pick_pose[0],pick_pose[1])
        time.sleep(0.1)
        self.gripper.open()
        time.sleep(0.1)
        self.robot.movel(pre_dive_pose[0],pre_dive_pose[1])

    def leave_object(self):
        self.robot.movel_rel([0, 0, 0.20])

    def grip(self, from_side):
        grip_centered = False
        self.gripped_object = OBJ_UNSET

        while(not grip_centered and self._abort_demo==False):
            detect_f = self.gripper.detect()

            if(detect_f == "none"):
                self.gripper.open()
                self.gripper.await_motion()
                continue

            ## Center correction along gripper y-axis: ##
            # y_correction = 0
            # if(detect_f=="f1"):
            #     y_correction = 0.01
            # elif(detect_f=="f2"):
            #     y_correction = -0.01
            
            # if(y_correction!=0):
            #     if(from_side==SIDE_RIGHT):
            #         self.robot.speedl(y_correction, 0, 0, timeout=2, blocking=False)
            #     elif(from_side==SIDE_LEFT):
            #         self.robot.speedl(-y_correction, 0, 0, timeout=2, blocking=False)
            #     elif(from_side==SIDE_BACK):
            #         self.robot.speedl(0, -y_correction, 0, timeout=2, blocking=False)

            # self.gripper.close(force=0.1)
            # self.robot.stopl()

            self.gripper.close(force=2)

            char = self.gripper.characterize_object()
            if(abs(char["cnt"]) <= 5):
                self.gripped_object = char["obj"]
                break
            else:
                z_correction = 1.0 * char["cnt"]
                self.gripper.gorel(15)
                if(from_side==SIDE_RIGHT):
                    self.robot.movel_rel([0, z_correction/1000, 0])
                elif(from_side==SIDE_LEFT):
                    self.robot.movel_rel([0, -z_correction/1000, 0])
                elif(from_side==SIDE_BACK):
                    self.robot.movel_rel([z_correction/1000, 0, 0])

        if(self.gripped_object==OBJ_UNKNOWN):
            self.gripper.release(max_change=8)
        else:
            self.gripper.hold_object()

    def pick(self, pick_pos, from_side):
        self.pick_from_side(pick_pos, from_side)
        attemps = 0
        while(attemps<=3):
            self.grip(from_side)
            # if(self.gripped_object == OBJ_UNSET):
            #     print("PICK UNSET")
            #     self.leave_object()
            #     return False
            if(self.gripped_object == OBJ_UNKNOWN):
                print("PICK UNKNOWN")
                self.gripper.detect(speed=10) # TODO handle result

                self.robot.speedl(0,0,0.01,timeout=6, blocking=False)
                critical_force = self.gripper.slip_grip(return_when_no_slip=1.0)
                print(f"critical_force={critical_force}")
                self.gripper.hold_force(1.1*critical_force + 1.0)
                self.robot.stopl()
                return True

                # if(self.gripped_object == OBJ_NONE):
                #     self.gripper.open()
                #     self.gripper.await_motion()
                #     attemps+=1
                #     continue
                # else:
                #     self.robot.speedl(0,0,0.01,timeout=3)
                #     return True

                # self.leave_object()
                # return False
            # elif(self.gripped_object == OBJ_NONE):
            #     print("PICK NONE")
            #     self.gripper.await_motion()
            #     attemps+=1
            #     continue
                # self.leave_object()
                # return False
            else:
                return True

        self.leave_object()
        return False

    def pick_and_place(self, near_far): # right side to left side

        if(self.current_side==SIDE_BACK):
            self.prepare_right_grip()

        self.current_side=SIDE_RIGHT

        if(near_far==POS_NEAR):
            current_pos=pos_right_near
        elif(near_far==POS_FAR):
            current_pos=pos_right_far
        else:
            return False

        time.sleep(0.2)

        if( self.pick(current_pos, self.current_side) == False ):
            return False

        time.sleep(0.2)

        if(self.gripped_object == OBJ_EMPTY_CAN or self.gripped_object == OBJ_TOMATO or self.gripped_object == OBJ_MANDARIN):
            self.lift_from_side(current_pos, self.current_side)

            if(near_far == POS_NEAR):
                self.current_side=SIDE_BACK
                current_pos=pos_left_far
                self.prepare_back_grip()
                self.place_from_side(current_pos, self.current_side)

            elif(near_far == POS_FAR):
                self.current_side=SIDE_RIGHT
                current_pos=pos_left_far
                self.place_from_side(current_pos, self.current_side)

            else:
                raise "Unknown side"

            return True

        # elif(self.gripped_object == OBJ_TOMATO or self.gripped_object == OBJ_MANDARIN):
        #     self.lift_from_side(current_pos, self.current_side)
        #     self.place_from_side(current_pos, self.current_side)
        #     return False

        # elif(self.gripped_object == OBJ_FULL_CAN):
        else:
            self.lift_from_side(current_pos, self.current_side)
            self.current_side=SIDE_RIGHT
            current_pos=pos_left_near
            self.place_from_side(current_pos, self.current_side)
            return True    

        # else:
        #     self.lift_from_side(current_pos, self.current_side)
        #     self.present_gripper()

    def pick_and_place2(self, near_far, swap): # left side to right side
        print("pick_and_place2 prepare")
        if(near_far==POS_NEAR):
            current_pos=pos_left_near
            self.current_side=SIDE_RIGHT
        elif(near_far==POS_FAR):
            current_pos=pos_left_far
            self.current_side=SIDE_BACK
            self.prepare_back_grip()
        else:
            return False

        time.sleep(0.2)

        print("pick_and_place2 pick")
        if( self.pick(current_pos, self.current_side) == False ):
            return False

        time.sleep(0.2)

        # self.pick(current_pos, self.current_side)
        # if(self.gripped_object == OBJ_UNKNOWN or self.gripped_object == OBJ_UNSET):
        #     print("Unknown object!!")
        #     return
        #     # TODO

        print("pick_and_place2 lift")
        self.lift_from_side(current_pos, self.current_side)
        
        print("pick_and_place2 prepare 2")
        if(self.current_side == SIDE_BACK):
            self.prepare_right_grip()

        self.current_side = SIDE_RIGHT
        if(swap):
            if(near_far==POS_NEAR):
                current_pos=pos_right_far
            elif(near_far==POS_FAR):
                current_pos=pos_right_near
        else:
            if(near_far==POS_NEAR):
                current_pos=pos_right_near
            elif(near_far==POS_FAR):
                current_pos=pos_right_far

        print("pick_and_place2 place")
        self.place_from_side(current_pos, self.current_side)            
        return True

    def home(self):
        self.gripper.lights(False)
        self.gripper.open(blocking=False)
        self.robot.move_home()

    def run1(self, reset=True):
        print("run1")
        self._abort_demo=False
        self._demo_started=MAIN_DEMO
        self._demo_running=True

        self._callback(CBEVENT_DEMO_STARTED)

        self.robot.move_home()
        self.gripper.open()

        if(reset): 
            self.run_i = 0
            self.demo_run_state = 0
            
        while(self._abort_demo==False):
            if(self.demo_run_state == 0): # both cans at starting positions
                # move can 1
                succes = self.pick_and_place(POS_NEAR)
            elif(self.demo_run_state == 1): # 1st can moved
                # move can 2
                succes = self.pick_and_place(POS_FAR)
                self.robot.move_home()
            elif(self.demo_run_state == 2): # 2nd can moved
                # return can 1
                swap_cans = (self.run_i%2 == 0)
                succes = self.pick_and_place2(POS_NEAR, swap_cans)
            elif(self.demo_run_state == 3): # 1st can returned
                # return can 2
                swap_cans = (self.run_i%2 == 0)
                succes = self.pick_and_place2(POS_FAR,  swap_cans)
                
                self.robot.move_home()
                time.sleep(2)

            if(succes and self._abort_demo==False):
                self.demo_run_state += 1
                if(self.demo_run_state==4):
                    self.demo_run_state=0
                    self.run_i += 1
            else:
                self.robot.move_home()
                break

            time.sleep(0.2)

        self._demo_running=False
        self._callback(CBEVENT_DEMO_ENDED)

        # self.robot.move_home()

    # def test_speedl(self):
    #     self.robot.movel(pre_dive_1_pos, pre_dive_1_rot)
    #     time.sleep(1)
    #     self.robot.speedl(0,0.01,0,timeout=3)

    def run_SlipDemo(self):
        self._demo_started=SLIP_DEMO
        self._demo_running=True
        self._abort_demo=False
        self._callback(CBEVENT_DEMO_STARTED)

        self.gripper.open()
        self.present_gripper()

        while(self._abort_demo==False):
            self.gripper.slip_demo()
            time.sleep(0.5)

        self._demo_running=False
        self._callback(CBEVENT_DEMO_ENDED)

    def run_GripDemo(self):
        self._demo_started=GRIP_DEMO
        self._demo_running=True
        self._abort_demo=False
        self._callback(CBEVENT_DEMO_STARTED)

        self.gripper.open()
        self.present_gripper()

        self.gripper.grip_demo()

        while(self._abort_demo==False):
            time.sleep(0.1)

        self._demo_running=False
        self._callback(CBEVENT_DEMO_ENDED)

    def present_gripper(self):
        self.robot.movej(joints_present,spd=0.5)

if(__name__=="__main__"):
    robot = RobotInterface()
    robot.connect()

    gripper = GripperInterface("/dev/ttyACM0")

    demo = Demo(robot, gripper)
    demo.run1()

    #demo.test_speedl()

    # robot.move_home()
    # demo.pick_from_side(pos_right_near, side=SIDE_RIGHT)
    # robot.move_home()
    # demo.pick_from_side(pos_right_far, side=SIDE_RIGHT)
    # robot.move_home()
    # demo.pick_from_side(pos_left_near, side=SIDE_LEFT)
    # robot.move_home()
    # demo.pick_from_side(pos_left_far, side=SIDE_LEFT)
    # robot.move_home()
    
    print("Run done")

    time.sleep(1)
    robot.shutdown()
    gripper.shutdown()