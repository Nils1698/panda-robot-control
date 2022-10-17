import time
import tkinter as tk
from tkinter import *
from tkinter import ttk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import Mark10Controller 
import GUI_tools as gui
import numpy as np

MIN_CTRL_SPD = 0.5  # mm/min
MAX_CTRL_SPD = 1100 # mm/min

MAX_CTRL_ACC = 20 # (mm/min)/s

THROUGH_STAND = "Through stand"

CTRL_SPD_DEFAULT_VAL = 500

import os
base_folder = os.path.dirname(__file__)

class ControlFrame(ttk.Frame):
    def __init__(self, win, controller : Mark10Controller.Controller):
        super().__init__(win)
        self.win = win
        self.controller = controller

        self.controller.subscribe(self.controller_callback)

        self.needs_ui_update=False
        self.initUI()
        self.updateUI()

        self.updateForceLabel() # TODO move to connect_gauge
        self.updatePositionLabel() # TODO move to connect_stand

        self.var_ctrl_vel.set(0)
        self.checkNeedsUpdate()

    def controller_callback(self):
        self.needs_ui_update=True

    def stand_port_changed(self,port):
        if(port==gui.PortSelector.ENTRY_DISCONNECT):
            self.controller.disconnect()
        else:
            print("Open stand at "+port)
            self.controller.connect_stand(port)

    def gauge_port_changed(self,port):
        if(port==THROUGH_STAND):
            self.controller.connect_gauge(Mark10Controller.CONN_THROUGH_STAND)
        else:
            print("Open gauge at "+ port)
            self.controller.connect_gauge(port)

    def zero_force_clicked(self):
        self.controller.zero_force()
        self.updateUI()

    def ctrl_up_clicked(self):
        self.controller.go_up(speed=self.ent_speed.getValue())
        self.updateUI()

    def ctrl_down_clicked(self):
        self.controller.go_down(speed=self.ent_speed.getValue())
        self.updateUI()

    def ctrl_stop_clicked(self):
        self.controller.go_stop()
        self.updateUI()

    def ctrl_goto_clicked(self):
        if(self.ent_goto.isValid()):
            self.controller.goto(self.ent_goto.getValue(),relative=False)
            self.updateUI()

    def reset_pos_clicked(self):
        self.controller.zero_position()

    def ctrl_uprel_clicked(self):
        if(self.ent_gorel.isValid()):
            self.controller.goto(self.ent_gorel.getValue(),relative=True)
            self.updateUI()

    def ctrl_downrel_clicked(self):
        if(self.ent_gorel.isValid()):
            self.controller.goto(-self.ent_gorel.getValue(),relative=True)
            self.updateUI()

    def ctrl_vel_changed(self,var):
        self.controller.command_velocity(var.get())
        # self.controller.velocity_control_set_target(var.get())
        self.lbl_ctrl_vel['text'] = f"{var.get():.1f}"

    def ctrl_vel_released(self, event):
        self.decrease_ctrl_vel()

    def decrease_ctrl_vel(self):
        DELTA_T = 10 # ms
        MAX_CTRL_SPD_CHANGE = MAX_CTRL_ACC * DELTA_T

        if(self.var_ctrl_vel.get() > 0):
            self.var_ctrl_vel.set( max(0, self.var_ctrl_vel.get()-MAX_CTRL_SPD_CHANGE ) )
        elif(self.var_ctrl_vel.get() < 0):
            self.var_ctrl_vel.set( min(0, self.var_ctrl_vel.get()+MAX_CTRL_SPD_CHANGE ) )            

        if(self.var_ctrl_vel.get() != 0):
            self.after(DELTA_T, self.decrease_ctrl_vel)

    def use_hilimit_changed(self,*_):
        if(not self.ent_hilimit.isValid()):
            self.var_use_hilimit.set(0)

        if(self.var_use_hilimit.get()==1):
            self.controller.set_upper_limit(self.ent_hilimit.getValue())
        else:
            self.controller.set_upper_limit(500)

    def use_lolimit_changed(self,*_):
        if(not self.ent_lolimit.isValid()):
            self.var_use_lolimit.set(0)

        if(self.var_use_lolimit.get()==1):
            self.controller.set_lower_limit(self.ent_lolimit.getValue())
        else:
            self.controller.set_lower_limit(-500)

    def hilimit_changed(self,_):
        if(self.var_use_hilimit.get()==1): self.var_use_hilimit.set(0)

    def lolimit_changed(self,_):
        if(self.var_use_lolimit.get()==1): self.var_use_lolimit.set(0)

    def apply_force_clicked(self):
        steps=self.ent_force_steps.getArray()
        times=self.ent_force_time.getArray()

        if(steps.size==0 or times.size==0):
            return

        if(times.size==1 and steps.size>1):
            times = np.repeat(times,steps.size)

        if(steps.size!=times.size):
            print("Steps and times must have same number of values")
            return

        print(self.var_hold_force_pos)
        self.controller.apply_forces(steps, times, active_hold=(self.var_hold_force_pos.get()=="force"))
        self.updateUI()

    def cycles_show_clicked(self):
        self.tabControl.select(0)

    def apply_cycles_control_clicked(self):
        if(self.ent_cycles_force.isValid() and self.ent_cycles_incr.isValid() and self.ent_cycles_period.isValid()
            and self.ent_cycles_period2.isValid() and self.ent_cycles.isValid()):

            self.controller.apply_force_cycles(self.ent_cycles_force.getValue(), self.ent_cycles_period.getValue(), 
                                                self.ent_cycles_period2.getValue(), self.ent_cycles.getValue(), self.ent_cycles_incr.getValue(),
                                                active_hold=(self.var_hold_force_pos.get()=="force"))

        self.updateUI()

    def apply_comp_control_clicked(self):
        if(self.controller.current_force() > 0.001):
            print("Warning: compliance control - force not 0!")

        if(self.ent_comp_maxforce.isValid() and self.ent_comp_maxsqueeze.isValid() and self.ent_comp_speed.isValid()):
            self.controller.test_compliance(self.ent_comp_maxforce.getValue(), self.ent_comp_maxsqueeze.getValue(), self.ent_comp_speed.getValue())
            self.updateUI()

    def apply_dyn_control_clicked(self):
        self.controller.start_dynamic_control(self.ent_dyn_force.getValue(), self.ent_dyn_speed.getValue(), self.ent_dyn_period.getValue(), self.ent_dyn_cycles.getValue())
        self.updateUI()

    def approach_control_clicked(self):
        self.controller.approach()

    def release_control_clicked(self):
        self.controller.release()

    def force_steps_changed(self,_):
        steps = self.ent_force_steps.getArray()
        times = self.ent_force_time.getArray()
        if(steps.size>0 and (steps.size==times.size or times.size==1)):
            if(times.size==1): times=np.repeat(times, steps.size)
            times = np.insert(times,0,0)
            times = np.cumsum(times)
            x = np.repeat(times,2)
            steps = np.repeat(steps,2)
            steps = np.insert(steps,0,0)
            y = np.append(steps,0)
            self.force_graph.set_data(x,y)
            x_size = max(1,times.max())
            self.ax.set_xlim((-x_size*0.15, x_size*1.15))
            self.ax.set_ylim((0,steps.max()*1.15))
        else:
            self.force_graph.set_data([],[])

        self.canvas.draw()

    def updateForceLabel(self):
        self.lbl_force["text"] = "{:.3f}".format(self.controller.current_force())
        self.after(100, self.updateForceLabel)

    def updatePositionLabel(self):
        self.lbl_pos["text"] = "{:.2f}".format(self.controller.current_position())
        self.after(100, self.updatePositionLabel)

    def updateMaxForceLabel(self,_):
        maxforce = self.ent_cycles_force.getValue()+(self.ent_cycles.getValue()-1)*self.ent_cycles_incr.getValue()
        self.lbl_cycles_maxforce["text"] = str(maxforce)
        if(maxforce > Mark10Controller.FORCE_MAX_LIMIT):
            self.lbl_cycles_maxforce["fg"]='red'
        else:
            self.lbl_cycles_maxforce["fg"]='black'

    def checkNeedsUpdate(self):
        if(self.needs_ui_update): self.updateUI()
        self.after(100,self.checkNeedsUpdate)

    def updateUI(self):
        self.lbl_con_status["text"] = self.controller.error_string()

        ctrl_ready = self.controller.control_is_ready()

        gui.set_enabled(self.btn_zero_force, ctrl_ready)

        gui.set_enabled(self.btn_ctrl_up,   ctrl_ready)
        gui.set_enabled(self.btn_ctrl_stop, self.controller.is_ok())
        gui.set_enabled(self.btn_ctrl_down, ctrl_ready)

        if(ctrl_ready):
            self.sld_ctrl_vel.state(["!disabled"])
        else:
            self.sld_ctrl_vel.state(["disabled"])

        gui.set_enabled(self.ent_hilimit, ctrl_ready)
        gui.set_enabled(self.ent_lolimit, ctrl_ready)

        gui.set_enabled(self.btn_reset_pos,     ctrl_ready)
        gui.set_enabled(self.btn_goto,          ctrl_ready)
        gui.set_enabled(self.btn_gorel_down,    ctrl_ready)
        gui.set_enabled(self.btn_gorel_up,      ctrl_ready)

        gui.set_enabled(self.btn_apply_control, ctrl_ready)
        gui.set_enabled(self.btn_apply_cycles_control, ctrl_ready)
        gui.set_enabled(self.btn_apply_comp_control, ctrl_ready)
        gui.set_enabled(self.btn_approach, ctrl_ready)
        gui.set_enabled(self.btn_release, ctrl_ready)
        self.needs_ui_update=False

    def initUI(self):

        self.pack(fill=BOTH, expand=1)

        conn_frame = ttk.Frame(self)
        conn_frame.grid(row=0,column=0,sticky=NSEW)
        conn_frame.rowconfigure(0,weight=0)
        conn_frame.columnconfigure(0,weight=1)
        self.rowconfigure(0,weight=0,minsize=30)

        gauge_frame = ttk.Frame(self)
        gauge_frame.grid(row=1,column=0,sticky=NSEW)
        self.rowconfigure(0,weight=1)
        self.rowconfigure(1,weight=2,minsize=60)

        ctrl_frame = ttk.Frame(self)
        ctrl_frame.grid(row=2,column=0,sticky=NSEW, padx=5)
        self.rowconfigure(2,weight=1)

        ## CONNECTION FRAME ##
        self.cmb_port = gui.PortSelector(conn_frame, "Stand: ", command=self.stand_port_changed, entries=[gui.PortSelector.ENTRY_DISCONNECT])
        self.cmb_port.grid(row=0,column=0,pady=(6,0),sticky=EW,padx=5)
        # gui.PortSelector(conn_frame, "Gauge: ", command=self.gauge_port_changed, entries=[THROUGH_STAND]).grid(row=1,column=0,sticky=EW,pady=(2,5),padx=5)

        self.lbl_stand_ok = Label(conn_frame, text="   ")
        self.lbl_stand_ok.grid(row=0,column=1, padx=5)
        self.lbl_gauge_ok = Label(conn_frame, text="   ")
        self.lbl_gauge_ok.grid(row=1,column=1, padx=5)

        self.lbl_con_status = Label(conn_frame, text="no response")
        self.lbl_con_status.grid(row=1, column=0, pady=(1,4))

        ## GAUGE FRAME ##
        gauge_frame.columnconfigure(0,weight=3)
        gauge_frame.columnconfigure(1,weight=0)
        gauge_frame.columnconfigure(2,weight=2)
        self.lbl_force = Label(gauge_frame, text="0.000", font=("Arial", 25), justify="center")
        self.lbl_force.grid(row=0,column=1, sticky=EW, pady=5)
        self.btn_zero_force = Button(gauge_frame, text="zero", width=1, command=self.zero_force_clicked)
        self.btn_zero_force.grid(row=0,column=2, sticky=W, pady=5)

        ## CONTROL FRAME ##
        ctrl_frame.rowconfigure(0, weight=0)
        Label(ctrl_frame, text="Speed",    font=("Arial", 14), anchor="center").grid(row=0,column=0, sticky=N, pady=(0,7))
        Label(ctrl_frame, text="Position", font=("Arial", 14), anchor="center").grid(row=0,column=1, sticky=N, pady=(0,7))
        ctrl_speed_frame = ttk.Frame(ctrl_frame)
        ctrl_speed_frame.grid(row=1,column=0,sticky=NSEW, padx=5)
        ctrl_pos_frame = ttk.Frame(ctrl_frame)
        ctrl_pos_frame.grid(row=1,column=1,sticky=NSEW)
        ctrl_frame.rowconfigure(1, weight=0)

        ctrl_frame.rowconfigure(2, weight=0)

        Label(ctrl_frame, text="Force Control", font=("Arial", 14)).grid(row=2,column=0, columnspan=2, sticky=S, pady=(20,5))
        ctrl_force_frame = ttk.Frame(ctrl_frame)
        ctrl_force_frame.grid(row=3,column=0,columnspan=2, sticky=NSEW, pady=(5,10))

        ctrl_frame.rowconfigure(3, weight=1)

        ### SPEED FRAME ###
        self.img_btn_up   = PhotoImage(file = os.path.join(base_folder, 'res/btn_up.png'))
        self.img_btn_down = PhotoImage(file = os.path.join(base_folder, 'res/btn_down.png'))
        self.img_btn_stop = PhotoImage(file = os.path.join(base_folder, 'res/btn_stop.png'))

        BUTTON_PAD = 5
        self.btn_ctrl_up = Button(ctrl_speed_frame, image=self.img_btn_up, command=self.ctrl_up_clicked)
        self.btn_ctrl_up.grid(row=0, column=0, pady=BUTTON_PAD)

        self.btn_ctrl_stop = Button(ctrl_speed_frame, image=self.img_btn_stop, command=self.ctrl_stop_clicked)
        self.btn_ctrl_stop.grid(row=1, column=0, pady=BUTTON_PAD)

        self.btn_ctrl_down = Button(ctrl_speed_frame, image=self.img_btn_down, command=self.ctrl_down_clicked)
        self.btn_ctrl_down.grid(row=2, column=0, pady=BUTTON_PAD)

        self.var_ctrl_vel = tk.DoubleVar()
        self.sld_ctrl_vel = ttk.Scale(ctrl_speed_frame, from_=MAX_CTRL_SPD, to_=-MAX_CTRL_SPD, orient="vertical", length=100, variable=self.var_ctrl_vel)
        self.sld_ctrl_vel.grid(row=0, column=1, rowspan=3, sticky=NS, padx=20, pady=BUTTON_PAD)
        self.sld_ctrl_vel.bind("<ButtonRelease-1>", self.ctrl_vel_released)
        self.var_ctrl_vel.trace("w", lambda name, index, mode, var=self.var_ctrl_vel: self.ctrl_vel_changed(var))

        self.ent_speed = gui.NumberEntry(ctrl_speed_frame, width=8, min_val=MIN_CTRL_SPD, max_val=MAX_CTRL_SPD, decimals=1, allow_negative=False, step_size=0.5, justify='center')
        self.ent_speed.insert(0,str(CTRL_SPD_DEFAULT_VAL))
        self.ent_speed.grid(row=3, column=0)

        self.lbl_ctrl_vel = Label(ctrl_speed_frame)
        self.lbl_ctrl_vel.grid(row=3,column=1)

        Label(ctrl_speed_frame, text="mm/min").grid(row=4,column=0)

        ### POS FRAME ###
        self.lbl_pos = Label(ctrl_pos_frame, font=("Arial", 14))
        self.lbl_pos.grid(row=1,column=0)
        self.btn_reset_pos = Button(ctrl_pos_frame, text="zero", width=2, command=self.reset_pos_clicked)
        self.btn_reset_pos.grid(row=1,column=1)

        self.ent_goto = gui.NumberEntry(ctrl_pos_frame, decimals=2, step_size=0.02, width=6, justify="center")
        self.ent_goto.grid(row=2, column=0)
        self.btn_goto = Button(ctrl_pos_frame, text="go", width=2, command=self.ctrl_goto_clicked)
        self.btn_goto.grid(row=2,column=1)

        GOREL_WIDTH = 6
        self.btn_gorel_up = Button(ctrl_pos_frame, text="up", width=GOREL_WIDTH-3, command=self.ctrl_uprel_clicked)
        self.btn_gorel_up.grid(row=7,column=0, columnspan=2, pady=(15,0))
        self.ent_gorel = gui.NumberEntry(ctrl_pos_frame, decimals=2, step_size=0.02, allow_negative=False, width=GOREL_WIDTH, justify="center")
        self.ent_gorel.grid(row=8, column=0, columnspan=2)
        self.btn_gorel_down = Button(ctrl_pos_frame, text="down", width=GOREL_WIDTH-3, command=self.ctrl_downrel_clicked)
        self.btn_gorel_down.grid(row=9,column=0, columnspan=2, pady=(0,20))

        Label(ctrl_pos_frame, text="Upper limit:").grid(row=11, column=0)
        self.ent_hilimit = gui.NumberEntry(ctrl_pos_frame, decimals=2, step_size=0.02, width=6, on_edit=self.hilimit_changed)
        self.ent_hilimit.grid(row=11, column=1)

        self.var_use_hilimit = IntVar()
        self.var_use_hilimit.trace_add("write", self.use_hilimit_changed)
        chk_use_hilimit = Checkbutton(ctrl_pos_frame, variable=self.var_use_hilimit)
        chk_use_hilimit.grid(row=11, column=2)

        Label(ctrl_pos_frame, text="Lower limit:").grid(row=12, column=0)
        self.ent_lolimit = gui.NumberEntry(ctrl_pos_frame, decimals=2, step_size=0.02, width=6, on_edit=self.lolimit_changed)
        self.ent_lolimit.grid(row=12, column=1, columnspan=1)

        self.var_use_lolimit = IntVar()
        self.var_use_lolimit.trace_add("write", self.use_lolimit_changed)
        chk_use_lolimit = Checkbutton(ctrl_pos_frame, variable=self.var_use_lolimit)
        chk_use_lolimit.grid(row=12, column=2)

        ### FORCE CONTORL FRAME ###
        self.tabControl = ttk.Notebook(ctrl_force_frame)
        self.tabControl.pack(expand=1, fill="both")

        step_ctrl_frame = ttk.Frame(self.tabControl)
        self.tabControl.add(step_ctrl_frame, text='Stepwise', pad=5)
        # Label(ctrl_force_frame, text="Type:").grid(row=0, column=0,sticky=EW)
        # self.cmb_control_type = ttk.Combobox(ctrl_force_frame,values=["Stepwise","Ramp"])
        # self.cmb_control_type.grid(row=0, column=1,sticky=EW)

        #### STEP CONTORL TAP ####
        step_ctrl_frame.rowconfigure(0, weight=0)
        step_ctrl_frame.rowconfigure(1, weight=0)
        step_ctrl_frame.rowconfigure(2, weight=1)
        step_ctrl_frame.columnconfigure(0, weight=1)
        step_ctrl_frame.columnconfigure(1, weight=1)

        Label(step_ctrl_frame, text="Steps [N]:").grid(row=0,column=0,padx=2,pady=(5,1))
        self.ent_force_steps = gui.ListEntry(step_ctrl_frame, val_type=float, allow_negative=False, on_edit=self.force_steps_changed)
        self.ent_force_steps.grid(row=0,column=1, sticky=EW, padx=2, pady=(5,1))

        Label(step_ctrl_frame, text="Time [s]:").grid(row=1,column=0,padx=2,pady=(1,5))
        self.ent_force_time = gui.ListEntry(step_ctrl_frame, val_type=float, allow_negative=False, on_edit=self.force_steps_changed)
        self.ent_force_time.grid(row=1,column=1, sticky=EW, padx=2, pady=(1,5))

        self.fig = Figure(figsize=(3,2))
        self.ax = self.fig.add_subplot(111)
        self.force_graph = self.ax.plot([], [], '-', ms=5, lw=1)[0]
        self.ax.set_xlim((0,1))
        self.ax.set_ylim((0,1))
        self.ax.grid()

        self.canvas = FigureCanvasTkAgg(self.fig, master=step_ctrl_frame)
        # self.canvas.get_tk_widget().pack(padx=30, pady=30)
        self.canvas.get_tk_widget().grid(row=2, column=0, columnspan=2, sticky=tk.NSEW, padx=10, pady=5)
        self.canvas.draw()

        # self.btn_apply_control = Button(step_ctrl_frame, text="Apply", command=self.apply_force_clicked)
        # self.btn_apply_control.grid(row=3,column=1, sticky=SE)
        
        step_ctrl_foot_frame = tk.Frame(step_ctrl_frame)
        step_ctrl_foot_frame.grid(row=3, column=0, columnspan=3, sticky=S+E+W, pady=(5,0))

        step_ctrl_foot_frame.columnconfigure(0,weight=1)
        step_ctrl_foot_frame.columnconfigure(1,weight=0)
        step_ctrl_foot_frame.columnconfigure(2,weight=0)

        self.var_hold_force_pos = tk.StringVar()
        Label(step_ctrl_foot_frame, text="Hold: ").grid(row=0,column=0,sticky=E)
        self.btn_step_hold_force = tk.Radiobutton(step_ctrl_foot_frame, text="Force", variable=self.var_hold_force_pos,indicatoron=False, value="force", width=8)
        self.btn_step_hold_force.grid(row=0, column=1, sticky=EW)
        self.btn_step_hold_pos   = tk.Radiobutton(step_ctrl_foot_frame, text="Position", variable=self.var_hold_force_pos,indicatoron=False, value="pos", width=8)
        self.btn_step_hold_pos.grid(row=0, column=2, sticky=W)
        self.var_hold_force_pos.set("force")

        step_ctrl_foot_frame.columnconfigure(3,weight=2)
        step_ctrl_foot_frame.columnconfigure(4,weight=1)

        self.btn_apply_control = Button(step_ctrl_foot_frame, text="Apply", command=self.apply_force_clicked)
        self.btn_apply_control.grid(row=0,column=4, sticky=SE, padx=(10,0))

        #### CYCLES TAB ####
        cycle_ctrl_frame = ttk.Frame(self.tabControl)
        self.tabControl.add(cycle_ctrl_frame, text='Cycles', pad=5)

        cycle_ctrl_frame.columnconfigure(0,weight=1)
        cycle_ctrl_frame.columnconfigure(1,weight=0)
        cycle_ctrl_frame.columnconfigure(2,weight=1)
        r=0
        Label(cycle_ctrl_frame, text="Force: ").grid(row=r,column=0,sticky=E)
        self.ent_cycles_force = gui.NumberEntry(cycle_ctrl_frame, allow_negative=False, min_val=Mark10Controller.FORCE_MIN_LIMIT, max_val=Mark10Controller.FORCE_MAX_LIMIT, width=6, on_edit=self.updateMaxForceLabel)
        self.ent_cycles_force.grid(row=r,column=1)
        Label(cycle_ctrl_frame, text="N").grid(row=r,column=2, sticky=W)
        r+=1
        Label(cycle_ctrl_frame, text="Increment: ").grid(row=r,column=0,sticky=E)
        self.ent_cycles_incr = gui.NumberEntry(cycle_ctrl_frame, allow_negative=False, width=6, on_edit=self.updateMaxForceLabel)
        self.ent_cycles_incr.grid(row=r,column=1)
        Label(cycle_ctrl_frame, text="N").grid(row=r,column=2, sticky=W)
        r+=1
        self.cmb_cycles_holdtime = ttk.Combobox(cycle_ctrl_frame, values=["Hold time","Step time"], width=8, state="readonly")
        self.cmb_cycles_holdtime.grid(row=r,column=0,sticky=E)
        self.cmb_cycles_holdtime.current(0)
        # Label(cycle_ctrl_frame, text="Hold period: ").grid(row=r,column=0,sticky=E)
        self.ent_cycles_period = gui.NumberEntry(cycle_ctrl_frame, allow_negative=False, width=6)
        self.ent_cycles_period.grid(row=r,column=1)
        Label(cycle_ctrl_frame, text="s").grid(row=r,column=2, sticky=W)
        r+=1
        self.cmb_cycles_holdtime = ttk.Combobox(cycle_ctrl_frame, values=["Off period","Cycle period"], width=8, state="readonly")
        self.cmb_cycles_holdtime.grid(row=r,column=0,sticky=E)
        self.cmb_cycles_holdtime.current(0)
        # Label(cycle_ctrl_frame, text="Off period: ").grid(row=r,column=0,sticky=E)
        self.ent_cycles_period2 = gui.NumberEntry(cycle_ctrl_frame, allow_negative=False, width=6)
        self.ent_cycles_period2.grid(row=r,column=1)
        Label(cycle_ctrl_frame, text="s").grid(row=r,column=2, sticky=W)
        r+=1
        Label(cycle_ctrl_frame, text="Cycles: ").grid(row=r,column=0,sticky=E)
        self.ent_cycles = gui.NumberEntry(cycle_ctrl_frame, value_type=int, min_val=1, allow_negative=False, width=6, on_edit=self.updateMaxForceLabel)
        self.ent_cycles.grid(row=r,column=1)
        r+=1
        Label(cycle_ctrl_frame, text="Max force: ").grid(row=r,column=0,sticky=E)
        self.lbl_cycles_maxforce = Label(cycle_ctrl_frame, text="0.0")
        self.lbl_cycles_maxforce.grid(row=r,column=1)
        Label(cycle_ctrl_frame, text="N").grid(row=r,column=2, sticky=W)

        # r+=1
        # self.btn_cycles_show = Button(cycle_ctrl_frame, text="Show", command=self.cycles_show_clicked)
        # self.btn_cycles_show.grid(row=r,column=0, columnspan=3, pady=(12,0))
        r+=1
        cycle_ctrl_frame.rowconfigure(r,weight=1)
        r+=1

        cycle_ctrl_foot_frame = tk.Frame(cycle_ctrl_frame)
        cycle_ctrl_foot_frame.grid(row=r, column=0, columnspan=3, sticky=S+E+W, pady=(5,0))

        cycle_ctrl_foot_frame.columnconfigure(0,weight=1)
        cycle_ctrl_foot_frame.columnconfigure(1,weight=0)
        cycle_ctrl_foot_frame.columnconfigure(2,weight=0)

        # Repeated in Step Control
        Label(cycle_ctrl_foot_frame, text="Hold: ").grid(row=0,column=0,sticky=E)
        self.btn_cycle_hold_force = tk.Radiobutton(cycle_ctrl_foot_frame, text="Force", variable=self.var_hold_force_pos,indicatoron=False, value="force", width=8)
        self.btn_cycle_hold_force.grid(row=0, column=1, sticky=EW)
        self.btn_cycle_hold_pos   = tk.Radiobutton(cycle_ctrl_foot_frame, text="Position", variable=self.var_hold_force_pos,indicatoron=False, value="pos", width=8)
        self.btn_cycle_hold_pos.grid(row=0, column=2, sticky=W)

        cycle_ctrl_foot_frame.columnconfigure(3,weight=2)
        cycle_ctrl_foot_frame.columnconfigure(4,weight=1)

        self.btn_apply_cycles_control = Button(cycle_ctrl_foot_frame, text="Apply", command=self.apply_cycles_control_clicked)
        self.btn_apply_cycles_control.grid(row=0,column=4, sticky=SE, padx=(10,0))

        #### COMPLIANCE TAB ####
        comp_ctrl_frame = ttk.Frame(self.tabControl)
        self.tabControl.add(comp_ctrl_frame, text='Compliance', pad=5)

        comp_ctrl_frame.columnconfigure(0,weight=1)
        comp_ctrl_frame.columnconfigure(1,weight=0)
        comp_ctrl_frame.columnconfigure(2,weight=1)
        Label(comp_ctrl_frame, text="Max force: ").grid(row=0,column=0,sticky=E)
        self.ent_comp_maxforce = gui.NumberEntry(comp_ctrl_frame, allow_negative=False, max_val=Mark10Controller.FORCE_MAX_LIMIT, width=6)
        self.ent_comp_maxforce.grid(row=0,column=1)
        Label(comp_ctrl_frame, text="N").grid(row=0,column=2, sticky=W)
        Label(comp_ctrl_frame, text="Max compression: ").grid(row=1,column=0,sticky=E)
        self.ent_comp_maxsqueeze = gui.NumberEntry(comp_ctrl_frame, allow_negative=False, width=6)
        self.ent_comp_maxsqueeze.grid(row=1,column=1)
        Label(comp_ctrl_frame, text="mm").grid(row=1,column=2, sticky=W)
        Label(comp_ctrl_frame, text="Speed: ").grid(row=2,column=0,sticky=E)
        self.ent_comp_speed = gui.NumberEntry(comp_ctrl_frame, min_val=MIN_CTRL_SPD, max_val=MAX_CTRL_SPD, decimals=1, allow_negative=False, step_size=0.5, width=6)
        self.ent_comp_speed.grid(row=2,column=1)
        Label(comp_ctrl_frame, text="mm/min").grid(row=2,column=2, sticky=W)

        # self.ent_comp_maxforce.setValue(1)
        # self.ent_comp_maxsqueeze.setValue(10)
        # self.ent_comp_speed.setValue(100)

        comp_ctrl_frame.rowconfigure(3,weight=1)

        self.btn_apply_comp_control = Button(comp_ctrl_frame, text="Apply", command=self.apply_comp_control_clicked)
        self.btn_apply_comp_control.grid(row=4,column=2, sticky=SE)

        #### GOTO TAB ####
        approach_ctrl_frame = ttk.Frame(self.tabControl)
        self.tabControl.add(approach_ctrl_frame, text='Goto', pad=5)

        approach_ctrl_frame.columnconfigure(0,weight=1)
        approach_ctrl_frame.columnconfigure(1,weight=0)
        approach_ctrl_frame.columnconfigure(2,weight=1)

        self.btn_approach = Button(approach_ctrl_frame, text="Approach", command=self.approach_control_clicked)
        self.btn_approach.grid(row=0,column=1)

        self.btn_release = Button(approach_ctrl_frame, text="Release", command=self.release_control_clicked)
        self.btn_release.grid(row=1,column=1)

        # #### DYNAMIC TAB ####
        # dyn_ctrl_frame = ttk.Frame(self.tabControl)
        # tabControl.add(dyn_ctrl_frame, text='Dynamic', pad=5)

        # dyn_ctrl_frame.columnconfigure(0,weight=1)
        # dyn_ctrl_frame.columnconfigure(1,weight=0)
        # dyn_ctrl_frame.columnconfigure(2,weight=1)
        # r = 0
        # Label(dyn_ctrl_frame, text="Force: ").grid(row=r,column=0,sticky=E)
        # self.ent_dyn_force = gui.NumberEntry(dyn_ctrl_frame, allow_negative=False, max_val=Mark10Controller.FORCE_MAX_LIMIT, width=6)
        # self.ent_dyn_force.grid(row=0,column=1)
        # Label(dyn_ctrl_frame, text="N").grid(row=r,column=2, sticky=W)
        # r+=1
        # Label(dyn_ctrl_frame, text="Period: ").grid(row=r,column=0,sticky=E)
        # self.ent_dyn_period = gui.NumberEntry(dyn_ctrl_frame, allow_negative=False, width=6)
        # self.ent_dyn_period.grid(row=r,column=1)
        # Label(dyn_ctrl_frame, text="s").grid(row=r,column=2, sticky=W)
        # r+=1
        # Label(dyn_ctrl_frame, text="Speed: ").grid(row=r,column=0,sticky=E)
        # self.ent_dyn_speed = gui.NumberEntry(dyn_ctrl_frame, min_val=MIN_CTRL_SPD, max_val=MAX_CTRL_SPD, decimals=1, allow_negative=False, step_size=0.5, width=6)
        # self.ent_dyn_speed.grid(row=r,column=1)
        # Label(dyn_ctrl_frame, text="mm/min").grid(row=r,column=2, sticky=W)
        # r+=1
        # Label(dyn_ctrl_frame, text="Cycles: ").grid(row=r,column=0,sticky=E)
        # self.ent_dyn_cycles = gui.NumberEntry(dyn_ctrl_frame, min_val=1, value_type=int, allow_negative=False, width=6)
        # self.ent_dyn_cycles.grid(row=r,column=1)

        # # self.ent_comp_maxforce.setValue(1)
        # # self.ent_comp_maxsqueeze.setValue(10)
        # # self.ent_comp_speed.setValue(100)
        # r+=1
        # dyn_ctrl_frame.rowconfigure(r,weight=1)

        # self.btn_apply_dyn_control = Button(dyn_ctrl_frame, text="Apply", command=self.apply_dyn_control_clicked)
        # self.btn_apply_dyn_control.grid(row=4,column=2, sticky=SE)


    def save_UI_state(self, db):
        db["stand_gui"] = {
            "cmb_port":self.cmb_port.get(),

            "ent_speed":self.ent_speed.get(),
            "ent_goto":self.ent_goto.get(),
            "ent_gorel":self.ent_gorel.get(),

            "tab_select":self.tabControl.select(),

            "ent_force_steps":self.ent_force_steps.get(),
            "ent_force_time":self.ent_force_time.get(),

            "ent_cycles_force":self.ent_cycles_force.get(),
            "ent_cycles_incr":self.ent_cycles_incr.get(),
            "ent_cycles_period":self.ent_cycles_period.get(),
            "ent_cycles_period2":self.ent_cycles_period2.get(),
            "ent_cycles":self.ent_cycles.get(),

            "ent_comp_maxforce":self.ent_comp_maxforce.get(),
            "ent_comp_maxsqueeze":self.ent_comp_maxsqueeze.get(),
            "ent_comp_speed":self.ent_comp_speed.get()
        }
        return db

    def load_UI_state(self, db):
        d = db["stand_gui"]
        if(d==None): return
        self.cmb_port.set_if_available(d["cmb_port"])

        gui.setText(self.ent_speed, d["ent_speed"])
        gui.setText(self.ent_goto, d["ent_goto"])
        gui.setText(self.ent_gorel, d["ent_gorel"])

        self.tabControl.select(d["tab_select"]),

        gui.setText(self.ent_force_steps, d["ent_force_steps"])
        gui.setText(self.ent_force_time, d["ent_force_time"])

        gui.setText(self.ent_cycles_force, d["ent_cycles_force"])
        gui.setText(self.ent_cycles_incr, d["ent_cycles_incr"])
        gui.setText(self.ent_cycles_period, d["ent_cycles_period"])
        gui.setText(self.ent_cycles_period2, d["ent_cycles_period2"])
        gui.setText(self.ent_cycles, d["ent_cycles"])

        gui.setText(self.ent_comp_maxforce, d["ent_comp_maxforce"])
        gui.setText(self.ent_comp_maxsqueeze, d["ent_comp_maxsqueeze"])
        gui.setText(self.ent_comp_speed, d["ent_comp_speed"])

if __name__ == "__main__":
    window = tk.Tk()
    window.title("Test Setup")
    window.style = ttk.Style()
    window.style.theme_use("alt")
    mark = Mark10Controller.Controller()
    ControlFrame(window, mark)
    window.mainloop()

    mark.shutdown()