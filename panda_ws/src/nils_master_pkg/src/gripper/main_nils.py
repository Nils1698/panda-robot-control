import sys
import os
import logging
from pathlib import Path

from ast import Str

from GripperInterface import *
from Sampler_GUI import *
from Sensor_GUI import *
from Stand_GUI import *
from Graph_GUI import *
from Calibrator_GUI import *
from os_interface import *

import tkinter as tk
import pickle

#Added from __main__.py
base_folder = os.path.dirname(__file__)
tmp_folder = os.path.join(base_folder, 'tmp')
Path(tmp_folder).mkdir(exist_ok=True)

logging.basicConfig(filename=os.path.join(tmp_folder, 'errors.log'), level=logging.ERROR)
logger = logging.getLogger(__name__)
handler = logging.StreamHandler(stream=sys.stdout)
logger.addHandler(handler)

class MainWindow(tk.Tk):
    def __init__(self, db_filename, backup_filename, appdata_dir=None):
        super().__init__()
        self.port = "/dev/ttyACM0"

        self.os_handle = OsInterface(self, appdata_dir)

        self.stand = Mark10Controller.Controller()

        self.sensor = SensorInterface.SensorInterface()
        self.sensor.input_signal_sep=","

        self.sampler = Sampler.Sampler(self.sensor, self.stand)

        self.current_profile = 1

        self.gripper = GripperInterface(self.port)

        if(self.sensor.isConnected()):
            self.sensor.disconnect()

        print("Clear monitor!")

        if(self.sensor.connect(self.port)):
            print("Port open!")
        else:
            print("Failed to open port")

        print("home gripper..")
        self.gripper.enable()
        #self.gripper.home(blocking=False)
        self.gripper.execute("m home", False)



    ###MIGHT NOT BE NEEDED###
    def on_closing(self):
        self.save_UI_state(self.db_filename)
        self.destroy()

        self.sensor.shutdown()
        self.stand.shutdown()
        self.sampler.shutdown()
if __name__ == "__main__":
    app = MainWindow(db_filename=os.path.join(tmp_folder, 'GUIstate_GraphMain'), backup_filename=os.path.join(tmp_folder, 'sampler.txt'), appdata_dir=tmp_folder)
