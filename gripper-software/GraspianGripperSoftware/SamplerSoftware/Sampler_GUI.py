import tkinter as tk
from tkinter import *
from tkinter import filedialog 
from tkinter import ttk
from tkinter.font import Font

import numpy as np
import pathlib

import GUI_tools as gui
import Mark10Controller
from os_interface import OsInterface
import SensorInterface
import Sampler

class SamplerFrame(ttk.Frame):
    PARAM_INDEX_SAMPLES = 0
    PARAM_INDEX_INTERVAL = 1
    PARAM_INDEX_DURATION = 2

    TAB_SINGLE   = "Single"
    TAB_DURATION = "Timed"
    TAB_AUTO     = "Auto"
    TAB_MONITOR  = "Monitor"

    AUTO_SAMP_OPTIONS = [
        {"name":"Control running", "start":Sampler.AUTO_TRIGGER_CONTROL_RUNNING, "pause":None,                              "stop":Sampler.AUTO_TRIGGER_CONTROL_RUNNING},
        {"name":"Force stable",    "start":Sampler.AUTO_TRIGGER_CONTROL_FLAG1,    "pause":Sampler.AUTO_TRIGGER_CONTROL_FLAG1, "stop":Sampler.AUTO_TRIGGER_CONTROL_RUNNING},
        {"name":"When force > x",       "start":Sampler.AUTO_TRIGGER_FORCE_GTX,       "pause":None,                              "stop":Sampler.AUTO_TRIGGER_CONTROL_RUNNING,  "x":Mark10Controller.DETECTION_FORCE},
        {"name":"While force > x",       "start":Sampler.AUTO_TRIGGER_FORCE_GTX,       "pause":None,                              "stop":Sampler.AUTO_TRIGGER_FORCE_GTX,  "x":Mark10Controller.DETECTION_FORCE},
        {"name":"Any signal > x",  "start":Sampler.AUTO_TRIGGER_ANY_SIGNAL_GTX,  "pause":None,                              "stop":Sampler.AUTO_TRIGGER_ANY_SIGNAL_GTX,   "x":0},
        {"name":"Active control",  "start":Sampler.AUTO_TRIGGER_CONTROL_FLAG1,    "pause":Sampler.AUTO_TRIGGER_CONTROL_FLAG1, "stop":Sampler.AUTO_TRIGGER_CONTROL_FLAG1},
        # {"name":"Signal x > 0",    "type":Sampler.AUTO_TRIGGER_SIGNAL_GTX,     "x":0}
    ]

    def __init__(self, master, sensor:SensorInterface.SensorInterface, sampler:Sampler.Sampler, os_handle:OsInterface, backup_file:str=None):
        super().__init__(master)

        self.sensor_interface = sensor
        self.sampler = sampler
        self.os_handle = os_handle
        self.backup_file = backup_file
        self.backup_written_lines = 0


        self.sensor_interface.subscribe(self.sensor_callback)
        self.sampler.subscribe(self.sampler_callback)

        self.samp_param_last_edit = np.array([1,2,3]) # lowest value corresponds to latest edit
        self.txt_sampling_state = DISABLED

        # self.parameters_ok = False
        self.needs_ui_update=False
        self.auto_options = self.AUTO_SAMP_OPTIONS.copy()
        self.initUI() # Note: triggers updateUI through 'bind'
        self.updateUI()
        self.checkNeedsUpdate()
        # self.updateDetectionLabel() # TODO start when opnening connection instead

    def sensor_callback(self,event):
        self.needs_ui_update=True

    def sampler_callback(self):
        self.needs_ui_update=True

    def tab_changed(self,_):
        self.updateUI()

    def auto_option_selected(self,_):
        self.updateUI()

    def autoX_changed(self,_):
        if("x" in self.current_auto_option()):
            self.current_auto_option()["x"] = self.ent_auto_x.getValue()

    def current_tab(self) -> str:
        return self.tabControl.tab(self.tabControl.select(), "text") 

    def current_auto_option(self) -> dict:
        return self.auto_options[self.cmb_auto.current()]

    def start_clicked(self):
        self.update_includes_from_UI()
        tab = self.current_tab()
        if(tab == self.TAB_SINGLE):     self.single_sample_clicked()
        elif(tab == self.TAB_DURATION): self.timed_sample_clicked()
        elif(tab == self.TAB_AUTO):     self.auto_sample_clicked()
        elif(tab == self.TAB_MONITOR):  self.start_monitor_clicked()

    def continue_clicked(self):
        self.sampler.continue_sampling()
        self.update_samples()
        self.updateUI()

    def stop_clicked(self):
        if(self.current_tab() == self.TAB_MONITOR): 
            self.sampler.pause_sampling()
        else:
            self.sampler.stop_sampling()
        self.updateUI()

    def single_sample_clicked(self):
        if(self.ent_single_x.get()==""):
            self.sampler.single_sample()
        else:
            self.sampler.single_sample(x=self.ent_single_x.getValue())

        self.update_samples()
        self.write_backup()
        self.updateUI()

    def timed_sample_clicked(self):
        self.auto_fill_sampling_params()
        if(self.ent_timed_samples.get()==""):
            self.sampler.start_timed_sampling(self.ent_timed_interval.getValue())
        else:
            self.sampler.start_timed_sampling(self.ent_timed_interval.getValue(), self.ent_timed_samples.getValue())
        self.start_sampling()
        self.updateUI()

    def auto_sample_clicked(self):
        self.sampler.start_auto_sampling(self.ent_auto_interval.getValue(), self.current_auto_option())
        self.start_sampling()

    def start_monitor_clicked(self):
        if(not self.ent_mon_lines.isValid()): self.ent_mon_lines.setValue(500)
        self.sampler.start_monitoring(self.ent_mon_interval.getValue(), self.ent_mon_lines.getValue())
        self.start_sampling()

    def start_sampling(self):
        self.clear_backup()
        self.update_samples()

    # def thres_slider_changed(self,_):
    #     val = self.sld_thres.get()
    #     self.lbl_thres["text"] = "{:.2f}".format(val)
    #     self.sensor_interface.detection_thres = val

    def reset_clicked(self):
        self.sampler.reset()
        self.txt_sampling['state'] = NORMAL
        self.txt_sampling.delete(1.0,"end")
        self.txt_sampling['state'] = self.txt_sampling_state
        self.updateUI()

    def allow_edit_clicked(self):
        if(self.var_allow_edit.get()==1):
            self.txt_sampling_state = NORMAL
        else:
            self.txt_sampling_state = DISABLED
        self.updateUI()

    def include_time_clicked(self,*_):
        self.sampler.set_include_time(self.var_incl_time.get()==1)

    def include_sensor_clicked(self,*_):
        self.sampler.set_include_sensor(self.var_incl_sensor.get()==1)

    def include_force_clicked(self,*_):
        self.sampler.set_include_force(self.var_incl_force.get()==1)

    def include_pos_clicked(self,*_):
        self.sampler.set_include_position(self.var_incl_pos.get()==1)

    def include_setpoint_clicked(self,*_):
        self.sampler.set_include_setpoint(self.var_incl_setpoint.get()==1)
 
    def update_includes_from_UI(self):
        self.sampler.set_include_time(self.var_incl_time.get()==1)
        self.sampler.set_include_sensor(self.var_incl_sensor.get()==1)
        self.sampler.set_include_force(self.var_incl_force.get()==1)
        self.sampler.set_include_position(self.var_incl_pos.get()==1)
        self.sampler.set_include_setpoint(self.var_incl_setpoint.get()==1)

    def export_clicked(self):
        fname = self.os_handle.save_csv_dialog()
        if(fname==None): return

        if(self.var_include_header.get()==1):
            print("Export with header")
            text2save = str(self.txt_sampling.get(1.0, END))
        else:
            print("Export without header")
            text2save = str(self.txt_sampling.get(2.0, END))

        with open(fname, "w") as f:
            f.write(text2save)

    def import_clicked(self):
        # TODO wrap in try-block
        fname = self.os_handle.load_csv_dialog()
        if(fname==None): return
        self.sampler.import_data(fname)
        self.update_samples()

    def entX_return_pressed(self,_):
        if(self.ent_single_x.get() != ""): self.single_sample_clicked()

    def set_sampling_paramter(self, entry, value):
        entry.config( validate = 'none')
        gui.setText(entry, "{0:.3g}".format(value))
        entry.config( validate = 'key')

    def sample_params_changed(self, entry : gui.NumberEntry):

        if(not entry.isValid()): # empty field
            self.samp_param_last_edit[entry.param_index] = np.max(self.samp_param_last_edit)+1
            self.updateUI()
            return

        self.samp_param_last_edit[entry.param_index] = 0
        self.samp_param_last_edit += 1

        self.auto_fill_sampling_params()
        self.updateUI()

    def auto_fill_sampling_params(self):
        if(not self.sampling_parameters_ok()):
            return

        sorted = np.argsort(self.samp_param_last_edit)
        edit_ent_i = sorted[-1]
        print("should edit: "+str(edit_ent_i))

        if(edit_ent_i==self.PARAM_INDEX_SAMPLES):
            samples = int( self.ent_timed_duration.getValue()/self.ent_timed_interval.getValue() )
            self.ent_timed_samples.setValue(samples, dont_callback=True)
        elif(edit_ent_i==self.PARAM_INDEX_INTERVAL):
                interval = self.ent_timed_duration.getValue() / self.ent_timed_samples.getValue()
                self.ent_timed_interval.setValue(interval, dont_callback=True)
        elif(edit_ent_i==self.PARAM_INDEX_DURATION):
                duration = self.ent_timed_samples.getValue() * self.ent_timed_interval.getValue()
                self.ent_timed_duration.setValue(duration, dont_callback=True)

    def sampling_parameters_ok(self):
        # At least 2 out of 3 specified
        return int(self.ent_timed_samples.isValid())+int(self.ent_timed_interval.isValid())+int(self.ent_timed_duration.isValid()) >= 2

    # def updateDetectionLabel(self):
    #     if(self.sensor_interface.detection):
    #         self.lbl_detection["text"] = "sample"
    #         self.lbl_detection["background"] = "green"
    #     else:
    #         self.lbl_detection["text"] = "stop"
    #         self.lbl_detection["background"] = "gray"

        # self.after(100, self.updateDetectionLabel)

    def update_samples(self):
        scroll_pos = self.txt_sampling.yview()

        self.txt_sampling['state'] = NORMAL
        self.txt_sampling.delete(1.0, "end")
        # self.txt_sampling.insert(1.0,self.sampler.sampling_output)
        samp_text = ', '.join(self.sampler.sampling_header())+"\n"
        samp_text += '\n'.join(', '.join('%0.3f'%x for x in y) for y in self.sampler.sampling_data())
        self.txt_sampling.insert(1.0,samp_text)
        self.txt_sampling['state'] = self.txt_sampling_state

        # self.write_backup()
        self.append_backup()

        if(scroll_pos[1]==1): # scrollbar at bottom
            self.txt_sampling.see('end')
        else:
            self.txt_sampling.yview_moveto(scroll_pos[0])

        if(not self.sampler.sampling_stopped()):
            self.after(100, self.update_samples)

    def append_backup(self):
        monitor_lines = int(self.txt_sampling.index('end-1c').split('.')[0])
        if(monitor_lines == self.backup_written_lines): return
        output = self.txt_sampling.get(f"{self.backup_written_lines}.0",END)
        if(self.backup_file != None):
            try:
                f = open(self.backup_file, "a")
                f.write(output)
                f.close()
                self.backup_written_lines = monitor_lines
            except OSError:
                print("Couldn't write backup file!")

    def clear_backup(self):
        if(self.backup_file != None):
            try:
                f = open(self.backup_file, "w")
                f.write("")
                f.close()
            except OSError:
                print("Couldn't write backup file!")

        self.backup_written_lines = 0
        pass

    def write_backup(self):
        output = str(self.txt_sampling.get(1.0, END))
        if(output==""): return
        if(self.backup_file != None):
            try:
                f = open(self.backup_file, "w")
                f.write(output)
                f.close()
            except OSError:
                print("Couldn't write backup file!")

    def checkNeedsUpdate(self):
        if(self.needs_ui_update): self.updateUI()
        self.after(100,self.checkNeedsUpdate)

    def set_chk_state(self, chk:Checkbutton, state):
        DEFAULT_DISABLED_COLOR = '#a3a3a3' # (grey)
        if(state=="unavailable"):
            chk['fg'] = DEFAULT_DISABLED_COLOR
            chk['activeforeground'] = 'grey'
            chk['disabledforeground'] = DEFAULT_DISABLED_COLOR
        elif(state=="ignored"):
            chk['fg'] = 'red'
            chk['activeforeground'] = 'red'
            chk['disabledforeground'] = '#8b6d70' # grey-red
        else:
            chk['fg'] = 'black'
            chk['activeforeground'] = 'black'
            chk['disabledforeground'] = DEFAULT_DISABLED_COLOR

    def set_chk_unavailable(self, chk:Checkbutton, unavail):
        if(unavail): self.set_chk_state(chk, "unavailable")
        else:        self.set_chk_state(chk, "normal")

    def set_chk_ignored(self, chk:Checkbutton, ignore):
        if(ignore and chk.var.get()==1):
            self.set_chk_state(chk, "ignored")
        else:
            self.set_chk_state(chk, "normal")

    def updateUI(self):
        incl_opt = self.sampler.get_include_options()

        # Tab: Single
        gui.set_enabled(self.ent_single_x,         self.sampler.state==Sampler.STATE_RESET or incl_opt[Sampler.INCL_OPT_X])

        # Timed
        gui.set_enabled(self.ent_timed_samples,    self.sampler.state==Sampler.STATE_RESET)
        gui.set_enabled(self.ent_timed_interval,   self.sampler.state==Sampler.STATE_RESET)
        gui.set_enabled(self.ent_timed_duration,   self.sampler.state==Sampler.STATE_RESET)

        # Tab: Auto
        gui.set_enabled(self.ent_auto_interval, self.sampler.state==Sampler.STATE_RESET)
        if("x" in self.current_auto_option()):
            self.ent_auto_x.setValue(self.current_auto_option()["x"])
            gui.set_enabled(self.ent_auto_x, self.sampler.state==Sampler.STATE_RESET)
        else:
            gui.setText(self.ent_auto_x, "")
            gui.set_enabled(self.ent_auto_x, False)

        # Tab: Monitor
        gui.set_enabled(self.ent_mon_interval,  self.sampler.state==Sampler.STATE_RESET)
        gui.set_enabled(self.ent_mon_lines,     self.sampler.state==Sampler.STATE_RESET)

        # Includes
        self.update_includes_from_UI() # 
        gui.set_enabled(self.chk_incl_time,     self.sampler.state==Sampler.STATE_RESET)
        gui.set_enabled(self.chk_incl_sensor,   self.sampler.state==Sampler.STATE_RESET)
        gui.set_enabled(self.chk_incl_force,    self.sampler.state==Sampler.STATE_RESET) 
        gui.set_enabled(self.chk_incl_pos,      self.sampler.state==Sampler.STATE_RESET)
        gui.set_enabled(self.chk_incl_setpoint, self.sampler.state==Sampler.STATE_RESET)

        if(self.sampler.state == Sampler.STATE_RESET):
            self.set_chk_unavailable(self.chk_incl_sensor,    not self.sampler.sensor_available())
            self.set_chk_unavailable(self.chk_incl_force,     not self.sampler.force_available())
            self.set_chk_unavailable(self.chk_incl_pos,       not self.sampler.position_available())
            self.set_chk_unavailable(self.chk_incl_setpoint,  not self.sampler.force_available())
        else:
            self.set_chk_ignored(self.chk_incl_sensor,    not self.sampler.get_include_option(Sampler.INCL_OPT_SENSOR))
            self.set_chk_ignored(self.chk_incl_force,     not self.sampler.get_include_option(Sampler.INCL_OPT_FORCE))
            self.set_chk_ignored(self.chk_incl_pos,       not self.sampler.get_include_option(Sampler.INCL_OPT_POS))
            self.set_chk_ignored(self.chk_incl_setpoint,  not self.sampler.get_include_option(Sampler.INCL_OPT_SETPOINT))

        # Tab control
        N_TABS = len(self.tabControl.tabs())
        if(self.sampler.state == Sampler.STATE_RESET):
            for i in range(N_TABS): self.tabControl.tab(i, state='normal')
        else:
            for i in range(N_TABS): 
                if(i==self.tabControl.index(self.tabControl.select())): continue
                self.tabControl.tab(i, state='disabled')

        # Start/clear Buttons
        if(self.current_tab()==self.TAB_SINGLE):
            gui.set_enabled(self.btn_sample, self.sampler.allow_single_sample())
            self.btn_sample["text"] = "Sample"
            self.btn_sample["command"] = self.start_clicked
        else:
            if(self.sampler.state == Sampler.STATE_RESET or self.sampler.state == Sampler.STATE_DONE):
                gui.set_enabled(self.btn_sample, self.sampler.allow_sampling())
                self.btn_sample["text"] = "Start"
                self.btn_sample["command"] = self.start_clicked
            else:
                gui.set_enabled(self.btn_sample, True)                
                if(self.sampler.state == Sampler.STATE_SAMPLING):
                    self.btn_sample["text"] = "Stop"
                    self.btn_sample["command"] = self.stop_clicked
                elif(self.sampler.state == Sampler.STATE_PAUSED):
                    self.btn_sample["text"] = "Continue"
                    self.btn_sample["command"] = self.continue_clicked

        # Footer
        empty_field = self.txt_sampling.compare("end-1c", "==", "1.0")
        can_export = self.sampler.sampling_stopped() and not empty_field
        gui.set_enabled(self.btn_import, self.sampler.state==Sampler.STATE_RESET or self.sampler.state == Sampler.STATE_DONE)
        gui.set_enabled(self.btn_reset,  can_export)
        gui.set_enabled(self.btn_export, can_export)
        gui.set_enabled(self.chk_include_header, can_export)
        gui.set_enabled(self.chk_allow_edit, self.sampler.sampling_stopped())
        
        self.txt_sampling['state'] = self.txt_sampling_state
        self.needs_ui_update=False

    def initUI(self):
        default_font = Font(family="Helvetica", size=gui.DEFAULT_FONT_SIZE)

        frame = self

        frame.columnconfigure(0, weight=1)

        frame.rowconfigure(0, weight=0)
        frame.rowconfigure(1, weight=1)
        frame.rowconfigure(2, weight=0)

        sampling_head2 = ttk.Frame(frame)
        sampling_head2.grid(row=0,column=0, padx=5, pady=5, sticky=N+S+E+W)

        sampling_body = ttk.Frame(frame)
        sampling_body.grid(row=1,column=0, padx=5, pady=2, sticky=N+S+E+W)

        sampling_foot = ttk.Frame(frame)
        sampling_foot.grid(row=2,column=0, padx=5, pady=5, sticky=N+S+E+W)

        # SAMPLING HEADER
        style = ttk.Style(self)
        style.configure('lefttab.TNotebook', tabposition='wn')
        self.tabControl = ttk.Notebook(sampling_head2, style='lefttab.TNotebook')
        self.tabControl.pack(expand=0, side=LEFT, fill='both', padx=5)
        self.tabControl.bind('<<NotebookTabChanged>>', self.tab_changed)

        single_sample_frame = ttk.Frame(self.tabControl)
        self.tabControl.add(single_sample_frame, text=self.TAB_SINGLE, pad=5)

        cont_sample_frame = ttk.Frame(self.tabControl)
        self.tabControl.add(cont_sample_frame, text=self.TAB_DURATION, pad=5)

        auto_sample_frame = ttk.Frame(self.tabControl)
        self.tabControl.add(auto_sample_frame, text=self.TAB_AUTO, pad=5)

        monitor_sample_frame = ttk.Frame(self.tabControl)
        self.tabControl.add(monitor_sample_frame, text=self.TAB_MONITOR, pad=5)

        head_include_frame = ttk.Frame(sampling_head2)
        head_include_frame.pack(expand=1, side=LEFT, fill='both', padx=5)

        ## Single ##

        # X
        r = 0; c = 0
        tk.Label(single_sample_frame,text="X:").grid(row=r,column=c, sticky=E)
        self.ent_single_x = gui.NumberEntry(single_sample_frame, width=5, value_type=float)
        self.ent_single_x.grid(row=r,column=c+1)
        self.ent_single_x.bind('<Return>', self.entX_return_pressed)

        ## Duration ##

        # Samples
        r = 0; c = 0
        tk.Label(cont_sample_frame,text="Samples:").grid(row=r,column=c, sticky=E)
        self.ent_timed_samples = gui.NumberEntry(cont_sample_frame, width=5, allow_negative=False, value_type=int, min_val=1, on_edit=self.sample_params_changed)
        self.ent_timed_samples.param_index = self.PARAM_INDEX_SAMPLES
        self.ent_timed_samples.grid(row=r,column=c+1, sticky=EW)

        r+=1
        # Interval
        tk.Label(cont_sample_frame,text="Interval:").grid(row=r,column=c, sticky=E)
        self.ent_timed_interval = gui.NumberEntry(cont_sample_frame, width=5, allow_negative=False, value_type=float, min_val=0.02, on_edit=self.sample_params_changed)
        self.ent_timed_interval.param_index = self.PARAM_INDEX_INTERVAL
        self.ent_timed_interval.grid(row=r,column=c+1, sticky=EW)
        tk.Label(cont_sample_frame,text="s").grid(row=r,column=c+2,padx=(0,7),sticky=W)

        r+=1
        # Duration
        tk.Label(cont_sample_frame,text="Duration:").grid(row=r,column=c, sticky=E)
        self.ent_timed_duration = gui.NumberEntry(cont_sample_frame, width=5, allow_negative=False, value_type=float, min_val=0.02, on_edit=self.sample_params_changed)
        self.ent_timed_duration.param_index = self.PARAM_INDEX_DURATION
        self.ent_timed_duration.grid(row=r,column=c+1, sticky=EW)
        tk.Label(cont_sample_frame,text="s").grid(row=r,column=c+2,sticky=W)

        ## Auto ##
        r = 0; c = 0
        tk.Label(auto_sample_frame,text="Interval:").grid(row=r,column=c, sticky=E)
        self.ent_auto_interval = gui.NumberEntry(auto_sample_frame, width=5, allow_negative=False, value_type=float, min_val=0.02)
        self.ent_auto_interval.grid(row=r, column=c+1)
        r+=1

        self.cmb_auto = ttk.Combobox(auto_sample_frame,width=14,values=list([d["name"] for d in self.AUTO_SAMP_OPTIONS]), state="readonly")
        self.cmb_auto.grid(row=r, column=c, columnspan=2, pady=(3,5), sticky=EW)
        self.cmb_auto.current(0)
        r+=1
        tk.Label(auto_sample_frame,text="x:").grid(row=r,column=c, sticky=E, padx=(0,4))
        self.ent_auto_x = gui.NumberEntry(auto_sample_frame, width=5, allow_negative=True, value_type=float, on_edit=self.autoX_changed)
        self.ent_auto_x.grid(row=r, column=c+1, sticky=EW)

        ## Monitor ##
        r = 0; c = 0
        tk.Label(monitor_sample_frame,text="Interval:").grid(row=r,column=c, sticky=E)
        self.ent_mon_interval = gui.NumberEntry(monitor_sample_frame, width=5, allow_negative=False, value_type=float, min_val=0.02)
        self.ent_mon_interval.grid(row=r, column=c+1)
        r+=1
        tk.Label(monitor_sample_frame,text="Max lines:").grid(row=r,column=c, sticky=E)
        self.ent_mon_lines = gui.NumberEntry(monitor_sample_frame, width=5, allow_negative=False, value_type=int, min_val=1)
        self.ent_mon_lines.grid(row=r, column=c+1, sticky=W)

        ### INCLUDE ###

        head_include_frame.columnconfigure(0,weight=2)
        head_include_frame.columnconfigure(1,weight=1)
        head_include_frame.columnconfigure(2,weight=1)
        head_include_frame.columnconfigure(3,weight=2)

        r=0; c=0
        Label(head_include_frame, text="Include", anchor="center").grid(row=r, column=c, columnspan=3, sticky=EW)

        r=1; c=1
        # Include time
        self.var_incl_time = IntVar()
        self.var_incl_time.trace_add('write', self.include_time_clicked)
        self.chk_incl_time = Checkbutton(head_include_frame, text="Time", variable=self.var_incl_time)
        self.chk_incl_time.grid(row=r,column=c, sticky=W)
        self.chk_incl_time.var = self.var_incl_time

        r+=1
        # Include sensor
        self.var_incl_sensor = IntVar()
        self.var_incl_sensor.trace_add('write', self.include_sensor_clicked)
        self.chk_incl_sensor = Checkbutton(head_include_frame, text="Sensor", variable=self.var_incl_sensor)
        self.chk_incl_sensor.grid(row=r,column=c, sticky=W)
        self.chk_incl_sensor.var = self.var_incl_sensor

        r=1; c=2
        # Include force
        self.var_incl_force = IntVar()
        self.var_incl_force.trace_add('write', self.include_force_clicked)
        self.chk_incl_force = Checkbutton(head_include_frame, text="Force", variable=self.var_incl_force)
        self.chk_incl_force.grid(row=r,column=c, sticky=W)
        self.chk_incl_force.var = self.var_incl_force

        r+=1
        # Include position
        self.var_incl_pos = IntVar()
        self.var_incl_pos.trace_add('write', self.include_pos_clicked)
        self.chk_incl_pos = Checkbutton(head_include_frame, text="Position", variable=self.var_incl_pos)
        self.chk_incl_pos.grid(row=r,column=c, sticky=W)
        self.chk_incl_pos.var = self.var_incl_pos

        r+=1
        # Include set point
        self.var_incl_setpoint = IntVar()
        self.var_incl_setpoint.trace_add('write', self.include_setpoint_clicked)
        self.chk_incl_setpoint = Checkbutton(head_include_frame, text="Set point", variable=self.var_incl_setpoint)
        self.chk_incl_setpoint.grid(row=r,column=c, sticky=W)
        self.chk_incl_setpoint.var = self.var_incl_setpoint

        # Sampling Button
        self.btn_sample = Button(sampling_head2,text="Start", width=8, command=self.start_clicked)
        self.btn_sample.pack(side=BOTTOM, anchor=SE, expand=0)

        # Clear Button
        self.btn_reset = Button(sampling_head2, text="Clear", width=8, command=self.reset_clicked)
        self.btn_reset.pack(side=BOTTOM, anchor=SE, expand=0)

        # self.btn_sample.grid(row=r,column=c, pady=1, sticky=SE)

        # # Slider
        # tk.Label(sampling_head,text="Threshold:").grid(row=2,column=1,sticky=E)
        # self.sld_thres = ttk.Scale(sampling_head, from_=0, to_=1, length=100, command=self.thres_slider_changed)
        # self.sld_thres.grid(row=2,column=2, columnspan=3, sticky=EW)
        # self.lbl_thres = tk.Label(sampling_head,text="0.0%")
        # self.lbl_thres.grid(row=2,column=5,sticky=W,padx=(2,0))
        # self.sld_thres.set(0.1)

        # self.lbl_detection = tk.Label(sampling_head,text="stop", background='gray', width=7)
        # self.lbl_detection.grid(row=2,column=6,columnspan=3, padx=(5,5), sticky=E)

        # # Calib Button
        # self.btn_auto_calib = Button(sampling_head,text="Calib", width=8, command=self.auto_calib_clicked)
        # self.btn_auto_calib.grid(row=2,column=9, columnspan=3, pady=1, sticky=E)

        # Auto Sampling Button
        # self.btn_auto_sample = Button(sampling_head,text="Auto sample", width=10, command=self.auto_sample_clicked)
        # self.btn_auto_sample.grid(row=2,column=12, pady=1)


        # SAMPLING BODY
        sampling_body.columnconfigure(0, weight=1)
        sampling_body.rowconfigure(0, weight=1)

        self.txt_sampling = tk.Text(sampling_body, width=60,height=20, state=DISABLED,font=default_font)
        self.txt_sampling.grid(column=0, row=0, sticky=tk.NSEW)

        sb_sampling = ttk.Scrollbar(sampling_body,
                                    orient='vertical',
                                    command=self.txt_sampling.yview)

        sb_sampling.grid(column=1, row=0, sticky=tk.NS)
        self.txt_sampling['yscrollcommand'] = sb_sampling.set


        # SAMPLING FOOTER
        self.btn_import = Button(sampling_foot, text="Import", command=self.import_clicked)
        self.btn_import.pack(side=LEFT, padx=(0,10))

        self.var_allow_edit = IntVar()
        self.chk_allow_edit = Checkbutton(sampling_foot, text="Allow editting", variable=self.var_allow_edit, command=self.allow_edit_clicked)
        self.chk_allow_edit.pack(side=LEFT, fill='none', expand=True, padx=10)

        self.btn_export = Button(sampling_foot, text="Export", command=self.export_clicked)
        self.btn_export.pack(side=RIGHT, padx=(0,10))

        self.var_include_header = IntVar()
        self.chk_include_header = Checkbutton(sampling_foot, text="Include header", variable=self.var_include_header)
        self.chk_include_header.pack(side=RIGHT, fill='none', expand=True, padx=(10,0))
        self.var_include_header.set(1)

        # Binds
        self.cmb_auto.bind('<<ComboboxSelected>>', self.auto_option_selected) 


    def save_UI_state(self, db):
        db["sampler_gui"] = {

            "var_incl_time":self.var_incl_time.get(),
            "var_incl_pos":self.var_incl_pos.get(),
            "var_incl_force":self.var_incl_force.get(),
            "var_incl_setpoint":self.var_incl_setpoint.get(),
            "var_incl_sensor":self.var_incl_sensor.get(),

            "tab_select":self.tabControl.select(),

            "ent_single_x":self.ent_single_x.get(),

            "ent_timed_samples":self.ent_timed_samples.get(),
            "ent_timed_interval":self.ent_timed_interval.get(),
            "ent_timed_duration":self.ent_timed_duration.get(),

            "ent_auto_interval":self.ent_auto_interval.get(),
            "cmb_auto":self.cmb_auto.get(),
            "ent_auto_x":self.ent_auto_x.get(),

            "ent_mon_interval":self.ent_mon_interval.get(),
            "ent_mon_lines":self.ent_mon_lines.get(),

        }
        return db

    def load_UI_state(self, db):
        d = db["sampler_gui"]
        if(d==None): return

        self.var_incl_time.set(d["var_incl_time"])
        self.var_incl_pos.set(d["var_incl_pos"])
        self.var_incl_force.set(d["var_incl_force"])
        self.var_incl_setpoint.set(d["var_incl_setpoint"])
        self.var_incl_sensor.set(d["var_incl_sensor"])
        self.update_includes_from_UI()

        self.tabControl.select(d["tab_select"]),

        gui.setText(self.ent_single_x, d["ent_single_x"])

        gui.setText(self.ent_timed_samples, d["ent_timed_samples"])
        gui.setText(self.ent_timed_interval, d["ent_timed_interval"])
        gui.setText(self.ent_timed_duration, d["ent_timed_duration"])

        gui.setText(self.ent_auto_interval, d["ent_auto_interval"])
        self.cmb_auto.set(d["cmb_auto"])
        gui.setText(self.ent_auto_x, d["ent_auto_x"])

        gui.setText(self.ent_mon_interval, d["ent_mon_interval"])
        gui.setText(self.ent_mon_lines, d["ent_mon_lines"])

        self.updateUI()

if __name__ == "__main__":
    window = tk.Tk()
    window.title("Sampler")
    window.style = ttk.Style()
    window.style.theme_use("alt")
    sensor = SensorInterface.SensorInterface()
    stand = Mark10Controller.Controller()
    sampler = Sampler.Sampler(sensor,stand)
    sf = SamplerFrame(window, sensor, sampler, OsInterface(window))
    sf.pack(fill=BOTH, expand=True)

    window.mainloop()
    sensor.shutdown()
    stand.shutdown()
    sampler.shutdown()