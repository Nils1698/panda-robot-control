from sqlite3 import Row
import tkinter as tk
from tkinter import ttk
from tkinter import *

import numpy as np

import GUI_tools as gui
import Graph_GUI, Sensor_GUI, Stand_GUI

CAL_TYPE_POLY2 = "Poly, deg 2"
CAL_TYPE_POLY3 = "Poly, deg 3"
CAL_TYPE_LINSPLINE = "Linear Spline"
CAL_TYPE_LINSPLINE_K = "Linear Spline, offset"
CAL_TYPE_LINSPLINE_ACT = "Linear Spline, activation"
CAL_TYPE_POLYSPLINE = "Poly Spline"
CAL_TYPE_POLYSPLINE_K = "Poly Spline, offset"
CAL_TYPE_POLYSPLINE2 = "Poly Spline (non-smooth)"
CAL_TYPES = [CAL_TYPE_POLY2,CAL_TYPE_POLY3,CAL_TYPE_LINSPLINE,CAL_TYPE_LINSPLINE_K, CAL_TYPE_LINSPLINE_ACT,CAL_TYPE_POLYSPLINE,CAL_TYPE_POLYSPLINE_K,CAL_TYPE_POLYSPLINE2]

class CalibratorFrame(ttk.Frame):

    def __init__(self, master, graph:Graph_GUI.GraphFrame, sensor:Sensor_GUI.SensorFrame, stand:Stand_GUI.ControlFrame):
        super().__init__(master)

        self.graph_gui = graph
        self.sensor_gui = sensor
        self.stand_gui = stand

        self.needs_ui_update=False

        self.graph_gui.sampler.subscribe(self.sampler_callback)

        self.initUI()
        self.updateUI()

    def sampler_callback(self):
        # self.needs_ui_update=True
        pass

    def combo_taxel_will_show(self,_):
        self.cmb_taxel_input["values"] = self.graph_gui.sampler.sampling_header()

    def updateUI(self):
        pass
        self.needs_ui_update=False

    def initUI(self):
        
        frame = self

        frame.columnconfigure(0, weight=1)

        frame.rowconfigure(0, weight=0)

        self.cmb_cal_type = ttk.Combobox(frame, width=18, values=CAL_TYPES, state="readonly")
        self.cmb_cal_type.grid(row=0, column=0)

        self.cmb_taxel_input = ttk.Combobox(frame, width=18, values=[], state="readonly")
        self.cmb_taxel_input.grid(row=1, column=0)
        self.cmb_taxel_input.bind("<Button-1>", self.combo_taxel_will_show)

# import SensorInterface

if __name__ == "__main__":
    window = tk.Tk()
    window.title("Calibrator")
    window.style = ttk.Style()
    window.style.theme_use("alt")

    # sensor = SensorInterface.SensorInterface()
    # sampler = Sampler.Sampler(sensor,None)
    
    frame = CalibratorFrame(window, None)
    frame.pack(fill=BOTH, expand=True)

    window.mainloop()

    # sensor.shutdown()
    # sampler.shutdown()