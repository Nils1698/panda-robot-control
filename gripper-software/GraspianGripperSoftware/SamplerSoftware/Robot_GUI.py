import tkinter as tk
from tkinter import *
from tkinter import ttk

from RobotInterface import RobotInterface, CBEVENT_ROBOT_PROTECTIVE_STOP
from GripperInterface import GripperInterface, CBEVENT_GRIPPER_OBJECT_DROPPED
import numpy as np

import GUI_tools as gui
from DemoExecution import MAIN_DEMO, Demo

import os

from threading import Thread

base_folder = os.path.dirname(__file__)

class RobotFrame(ttk.Frame):
    def __init__(self, win, robot:RobotInterface, gripper:GripperInterface):
        super().__init__(win)
        self.win = win
        self.robot = robot
        self.gripper = gripper
        self.demo = Demo(self.robot, self.gripper)

        self.robot.subscribe(self.robot_callback)
        self.gripper.subscribe(self.gripper_callback)
        self.demo.subscribe(self.demo_callback)

        self.initUI()
        self.updateUI()
        self.needs_ui_update=False
        self.checkNeedsUpdate()

    def demo_callback(self, event):
        self.needs_ui_update=True

    def robot_callback(self, event):
        if(event==CBEVENT_ROBOT_PROTECTIVE_STOP):
            self.gripper.disable()

        self.needs_ui_update=True

    def gripper_callback(self, event):
        # if(event==CBEVENT_GRIPPER_OBJECT_DROPPED):
        #     print("RobotGUI: object dropped")
        #     self.demo.abort(move_home=True, after_delay=1.5)

        self.needs_ui_update=True

    def checkNeedsUpdate(self):
        if(self.needs_ui_update): self.updateUI()
        self.after(100,self.checkNeedsUpdate)

    ## GUI actions ##

    def connect_clicked(self):
        self.robot.connect()

    def home_gripper_clicked(self):
        print("home gripper..")
        self.gripper.enable()
        self.gripper.home(blocking=False)

    def open_gripper_clicked(self):
        self.gripper.enable()
        self.gripper.open()

    def present_gripper_clicked(self):
        self.demo.present_gripper()

    def home_clicked(self):
        self.robot.enable()
        # self.demo_thread = Thread(target=self.robot.move_home, args=(True,)).start()
        self.demo_thread = Thread(target=self.demo.home).start()

    def run_clicked(self):
        self.robot.enable()
        self.gripper.enable()
        self.demo_thread = Thread(target=self.demo.run1).start()

    def continue_clicked(self):
        self.robot.enable()
        self.gripper.enable()
        self.demo_thread = Thread(target=self.demo.run1, args=(False,)).start()

    def stop_clicked(self):
        self.demo.abort()

    def slip_demo_clicked(self):
        self.robot.enable()
        self.gripper.enable()
        self.demo_thread = Thread(target=self.demo.run_SlipDemo).start()

    def grip_demo_clicked(self):
        self.robot.enable()
        self.gripper.enable()
        self.demo_thread = Thread(target=self.demo.run_GripDemo).start()
        

    ## GUI updates ##

    def update_connection_label(self):
        connection_str = "Robot is "
        if(self.robot.isConnected()):
            connection_str += "connected"
            # self.btn_connect["state"] = "disabled"
            self.btn_connect["text"] = "Re-connect robot"

            if(self.robot.isPoweredOn()):
                connection_str += " and powered on"
            else:
                connection_str += " but powered off"

        else:
            connection_str += "not connected"
            # self.btn_connect["state"] = "normal"
            self.btn_connect["text"] = "Connect robot"

        if(self.gripper.isConnected()):
            connection_str+="\n Gripper is connected"
        else:
            connection_str+="\n Gripper is not connected"

        self.lbl_connection["text"] = connection_str

    def updateUI(self):
        self.update_connection_label()

        ready_to_run = self.robot.isReady() and self.demo.isRunning()==False
        gui.set_enabled(self.btn_home, ready_to_run)
        gui.set_enabled(self.btn_home_gripper, self.gripper.isConnected())
        gui.set_enabled(self.btn_open_gripper, self.gripper.isConnected())
        gui.set_enabled(self.btn_demo1, ready_to_run)
        gui.set_enabled(self.btn_demo1_cont, ready_to_run and self.demo.currentDemo()==MAIN_DEMO)
        gui.set_enabled(self.btn_stop, self.demo.isRunning())

        gui.set_enabled(self.btn_slipdemo, ready_to_run)
        gui.set_enabled(self.btn_gripdemo, ready_to_run)

        self.needs_ui_update=False

    def initUI(self):
        SECTION_PADDING = (14,3)

        self.pack(fill=BOTH, expand=1)
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=0, minsize=200)
        self.columnconfigure(2, weight=1)
        fc = 1

        fr=0
        conn_frame = ttk.Frame(self)
        self.rowconfigure(fr,weight=0)
        conn_frame.grid(row=fr,column=fc,sticky=NSEW)

        fr+=1
        self.rowconfigure(fr,weight=1,minsize=20)

        fr+=1
        ctrl_frame = ttk.Frame(self)
        self.rowconfigure(fr,weight=1)
        ctrl_frame.grid(row=fr,column=fc,sticky=NSEW)

        fr+=1
        self.rowconfigure(fr,weight=3,pad=40)

        # Connection frame
        conn_frame.columnconfigure(0, weight=1)

        r=0; c=0

        Label(conn_frame, text="Initialize").grid(row=r, column=c, sticky=NSEW, pady=SECTION_PADDING)
        r+=1

        conn_frame.rowconfigure(r, weight=0)
        self.btn_connect = Button(conn_frame, text="Connect robot", command=self.connect_clicked)
        self.btn_connect.grid(row=r, column=c, sticky=NSEW)

        r+=1
        ctrl_frame.rowconfigure(r, weight=0)
        self.btn_home_gripper = Button(conn_frame, text="Home gripper", command=self.home_gripper_clicked)
        self.btn_home_gripper.grid(row=r, column=c, sticky=NSEW)

        r+=1
        conn_frame.rowconfigure(r, weight=0)
        self.lbl_connection = Label(conn_frame, text=" ... ")
        self.lbl_connection.grid(row=r, column=c, sticky=NSEW)

        # Control frame
        ctrl_frame.columnconfigure(0, weight=1)

        r+=1
        Label(ctrl_frame, text="Actions").grid(row=r, column=c, sticky=NSEW, pady=SECTION_PADDING)

        r+=1
        ctrl_frame.rowconfigure(r, weight=0)
        self.btn_open_gripper = Button(ctrl_frame, text="Open gripper", command=self.open_gripper_clicked)
        self.btn_open_gripper.grid(row=r, column=c, sticky=NSEW)

        r+=1
        ctrl_frame.rowconfigure(r, weight=0)
        self.btn_present = Button(ctrl_frame, text="Present gripper", command=self.present_gripper_clicked)
        self.btn_present.grid(row=r, column=c, sticky=NSEW)

        r+=1
        Label(ctrl_frame, text="Demo").grid(row=r, column=c, sticky=NSEW, pady=SECTION_PADDING)

        r+=1
        ctrl_frame.rowconfigure(r, weight=0)
        self.btn_demo1 = Button(ctrl_frame, text="Run demo", command=self.run_clicked)
        self.btn_demo1.grid(row=r, column=c, sticky=NSEW, pady=0)

        r+=1
        ctrl_frame.rowconfigure(r, weight=0)
        self.btn_demo1_cont = Button(ctrl_frame, text="Continue demo", command=self.continue_clicked)
        self.btn_demo1_cont.grid(row=r, column=c, sticky=NSEW, pady=0)

        r+=1
        ctrl_frame.rowconfigure(r, weight=0)
        self.btn_slipdemo = Button(ctrl_frame, text="Slip demo", command=self.slip_demo_clicked)
        self.btn_slipdemo.grid(row=r, column=c, sticky=NSEW, pady=(10,0))

        r+=1
        ctrl_frame.rowconfigure(r, weight=0)
        self.btn_gripdemo = Button(ctrl_frame, text="Grip demo", command=self.grip_demo_clicked)
        self.btn_gripdemo.grid(row=r, column=c, sticky=NSEW, pady=0)

        r+=1
        ctrl_frame.rowconfigure(r, weight=0)
        self.btn_stop = Button(ctrl_frame, text="Stop", command=self.stop_clicked)
        self.btn_stop.grid(row=r, column=c, sticky=NSEW, pady=(10,0))

        r+=1
        ctrl_frame.rowconfigure(r, weight=0)
        self.btn_home = Button(ctrl_frame, text="Move robot home", command=self.home_clicked)
        self.btn_home.grid(row=r, column=c, sticky=NSEW)


    def save_UI_state(self, db):
        return db

    def load_UI_state(self, db):
        pass

if __name__ == "__main__":
    robot = RobotInterface(autoconnect=False)
    gripper = GripperInterface("/dev/ttyUSB0")

    window = tk.Tk()
    window.title("Demo")
    window.style = ttk.Style()
    RobotFrame(window, robot, gripper)    
    window.mainloop()

    robot.shutdown()
    gripper.shutdown()
