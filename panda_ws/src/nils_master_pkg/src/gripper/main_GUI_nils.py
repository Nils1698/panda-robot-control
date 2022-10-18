import sys
import os
import logging
from pathlib import Path

from ast import Str
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

"""
Might not be needed
def handle_exception(exc_type, exc_value, exc_traceback):
    if issubclass(exc_type, KeyboardInterrupt):
        sys.__excepthook__(exc_type, exc_value, exc_traceback)
        return

    logger.error("Uncaught exception", exc_info=(exc_type, exc_value, exc_traceback))

sys.excepthook = handle_exception
"""


class MainWindow(tk.Tk):
    def __init__(self, db_filename, backup_filename, appdata_dir=None):
        super().__init__()
        #self.style = ttk.Style()
        #self.style.theme_use("alt")

        self.os_handle = OsInterface(self, appdata_dir)
        self.mainframe = ttk.Frame(self)

        self.stand = Mark10Controller.Controller()
        self.standFrame = ControlFrame(self.mainframe, self.stand)

        self.sensor = SensorInterface.SensorInterface()
        self.sensor.input_signal_sep=","
        self.sensorFrame = SensorFrame(self.mainframe, self.sensor)

        self.sampler = Sampler.Sampler(self.sensor, self.stand)
        self.samplerFrame = SamplerFrame(self.mainframe, self.sensor, self.sampler, self.os_handle, backup_file=backup_filename)

        self.graphCalibFrame = ttk.Frame(self.mainframe)        
        self.graphFrame = GraphFrame(self.graphCalibFrame, self.sampler, self.os_handle)
        self.calibFrame = CalibratorFrame(self.graphCalibFrame, self.graphFrame, self.sensorFrame, self.standFrame)

        self.current_profile = 1

        self.standFrame.shownVar  = tk.BooleanVar(value=False)
        self.samplerFrame.shownVar = tk.BooleanVar(value=True)
        self.sensorFrame.shownVar = tk.BooleanVar(value=True)
        self.graphFrame.shownVar  = tk.BooleanVar(value=True)
        self.calibFrame.shownVar  = tk.BooleanVar(value=False)
        self.menubar = self.createMenu()
        self.standFrame.grid(row=0,column=0); self.standFrame.grid_forget() # gives a layout manage error if removing this line
        self.layout_mainframe()

        self.db_filename = db_filename
        #self.load_UI_state(self.db_filename)

        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.bind('<Control-w>', lambda *_:self.on_closing())
        self.bind('<Alt-space>', lambda *_:self.toggle_menubar())

        self.os_handle.subscribe(self.export_name_changed)

    def export_name_changed(self):
        self.set_subtitle(self.os_handle.export_name)

    def set_subtitle(self, sub:Str):
        if(sub==""):
            self.title(self.MAIN_TITLE)
        else:
            self.title(self.MAIN_TITLE + " - " + sub)

    def load_UI_state(self, conf_file):
        print("Load UI state from "+conf_file)
        try:
            dbfile = open(conf_file, 'rb')     
            db = pickle.load(dbfile)
            self.os_handle.load_UI_state(db)
            self.sensorFrame.load_UI_state(db)
            self.standFrame.load_UI_state(db)
            self.samplerFrame.load_UI_state(db)
            self.graphFrame.load_UI_state(db)
            dbfile.close()
        except FileNotFoundError:
            print("State file not found")
        except KeyError:
            print("WARNING: Key error while loading state file!")
            dbfile.close()
        except Exception as e: 
            print(e)

    def save_UI_state(self, conf_file):
        try:
            state = {}
            self.os_handle.save_UI_state(state)
            self.sensorFrame.save_UI_state(state)
            self.standFrame.save_UI_state(state)
            self.samplerFrame.save_UI_state(state)
            self.graphFrame.save_UI_state(state)
            dbfile = open(conf_file, 'wb')
            pickle.dump(state, dbfile)
            dbfile.close()
        except FileNotFoundError:
            print("State file not found")
        except KeyError:
            print("WARNING: Key error while loading state file!")
            dbfile.close()
        except Exception as e: 
            print(e)

    def layout_mainframe(self):
        r=0; c=0
        if(self.standFrame.shownVar.get() ):
            self.mainframe.columnconfigure(c, weight=0)
            self.standFrame.grid(row=0,column=c,rowspan=2,sticky=NSEW)
            c+=1
        else: 
            self.standFrame.grid_forget()

        if(self.samplerFrame.shownVar.get() ):
            self.mainframe.columnconfigure(c, weight=1)
            self.samplerFrame.grid(row=0,column=c,sticky=NSEW)
            r+=1
        else: 
            self.samplerFrame.grid_forget()

        if(self.sensorFrame.shownVar.get() ):
            self.mainframe.columnconfigure(c, weight=1)
            self.mainframe.rowconfigure(0, weight=1)
            self.sensorFrame.grid(row=r,column=c,sticky=NSEW)
        else: 
            self.sensorFrame.grid_forget()

        if(self.graphFrame.shownVar.get() ):
            c+=1
            self.graphCalibFrame.columnconfigure(0, weight=1)
            self.graphCalibFrame.rowconfigure(0, weight=1)
            self.graphFrame.grid(row=0, column=0, sticky=NSEW)
            if(self.calibFrame.shownVar.get() ):
                self.graphCalibFrame.rowconfigure(1, weight=0)
                self.calibFrame.grid(row=1, column=0, sticky=NSEW)
            else:
                self.calibFrame.grid_forget()

            self.mainframe.columnconfigure(c, weight=1)
            self.graphCalibFrame.grid(row=0,column=c,rowspan=2,sticky=NSEW)
        else: 
            self.graphCalibFrame.grid_forget()

        self.mainframe.pack(fill=BOTH, expand=True)

    def populate_load_profiles(self):
        self.load_profile_menu.delete( 0, 'last' )
        profiles = self.os_handle.list_profiles()
        for i in range(len(profiles)):
            self.load_profile_menu.add_command(label=profiles[i][0], command=lambda filename=profiles[i][1]: self.load_UI_state(filename))

    def save_configuration(self):
        fn = self.os_handle.save_conf_dialog()
        if(fn):
            self.save_UI_state(fn)

    def createMenu(self):
        menubar = Menu(self)
        file_menu = Menu(menubar, tearoff=0)

        self.load_profile_menu = Menu(menubar, tearoff=0, postcommand=self.populate_load_profiles)
        file_menu.add_cascade(label="Load configuration", menu=self.load_profile_menu)

        save_profile_menu = Menu(menubar, tearoff=0)
        file_menu.add_command(label="Update configuration", command=None)
        file_menu.add_command(label="Save configuration...", command=self.save_configuration)
        for p in range(1,5):
            lbl = f"Profile {p}"
            if(p==self.current_profile): lbl = "*"+lbl
            save_profile_menu.add_command(label=lbl, command=None)

        file_menu.add_separator()
        file_menu.add_command(label="Export all...")
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.on_closing)
        menubar.add_cascade(label="File", menu=file_menu)

        view_menu = Menu(menubar, tearoff=0)
        view_menu.add_checkbutton(label="Stand", onvalue=1, offvalue=0, variable=self.standFrame.shownVar, command=self.layout_mainframe)
        view_menu.add_checkbutton(label="Sampler", onvalue=1, offvalue=0, variable=self.samplerFrame.shownVar, command=self.layout_mainframe)
        view_menu.add_checkbutton(label="Sensor", onvalue=1, offvalue=0, variable=self.sensorFrame.shownVar, command=self.layout_mainframe)
        view_menu.add_checkbutton(label="Graph", onvalue=1, offvalue=0, variable=self.graphFrame.shownVar, command=self.layout_mainframe)
        view_menu.add_checkbutton(label="Calibrator", onvalue=1, offvalue=0, variable=self.calibFrame.shownVar, command=self.layout_mainframe)
        menubar.add_cascade(label="View", menu=view_menu)

        return menubar

    def showMenu(self, do_show):
        if(do_show):
            self.config(menu=self.menubar)
        else:
            self.config(menu="") # hide menu

    def on_closing(self):
        self.save_UI_state(self.db_filename)
        self.destroy()

        self.sensor.shutdown()
        self.stand.shutdown()
        self.sampler.shutdown()

    def toggle_menubar(self):
            self.showMenu(self["menu"]=="")

if __name__ == "__main__":
    app = MainWindow(db_filename=os.path.join(tmp_folder, 'GUIstate_GraphMain'), backup_filename=os.path.join(tmp_folder, 'sampler.txt'), appdata_dir=tmp_folder)
    app.mainloop()