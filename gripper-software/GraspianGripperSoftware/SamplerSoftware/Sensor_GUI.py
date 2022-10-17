
import tkinter as tk
from tkinter import *
from tkinter import ttk
from tkinter.font import Font

import numpy as np
from numpy.lib.function_base import percentile
import GUI_tools as gui

import SensorInterface

# from sys import platform
# if platform == "linux" or platform == "linux2": 	# linux
# 	DEFAULT_PORT = '/dev/ttyACM0'
# elif platform == "darwin":     # OS X
# 	DEFAULT_PORT = '/dev/cu.usbmodemFD121'
# else:
# 	DEFAULT_PORT = 'COM3'  # Windows

MONITOR_REFRESH_DELAY = 100

CMD_LIST_MAX_SIZE = 10

class SensorFrame(ttk.Frame):

    def __init__(self, win, interface : SensorInterface.SensorInterface):
        super().__init__(win)
        self.win = win
        self.sensor_interface = interface
        
        self.sensor_interface.subscribe(self.sensor_callback)
        self.send_cmd_list = []
        self.send_cmd_current = ""
        self.send_cmd_index = 0

        self.needs_ui_update=False
        self.initUI()
        self.updateUI()
        self.update_monitor()
        self.checkNeedsUpdate()
        # self.port_sel.set_if_available(DEFAULT_PORT)

    def sensor_callback(self, event):
        if(event==SensorInterface.CB_CONNECTION_OPENED):
            print("Callback sensor connected!")
            self.selected_signals_changed(None)
        
        self.needs_ui_update=True

    def port_changed(self, port):
        if(port == gui.PortSelector.ENTRY_DISCONNECT):
            print("Close sensor interface!")
            if(self.sensor_interface.disconnect()):
                self.txt_monitor['state'] = NORMAL
                self.txt_monitor.insert(tk.END,"\n / Disconnected /")
                self.txt_monitor['state'] = DISABLED
                self.txt_monitor.see('end')
        else:
            if(self.sensor_interface.isConnected()):
                self.sensor_interface.disconnect()

            print("Clear monitor!")
            self.txt_monitor['state'] = NORMAL
            self.txt_monitor.delete("1.0", tk.END)
            self.txt_monitor['state'] = DISABLED

            if(self.sensor_interface.connect(port)):
                print("Port open!")
                self.updateUI()
            else:
                self.txt_monitor['state'] = NORMAL
                self.txt_monitor.insert(1.0,"Failed to open port\n")
                self.txt_monitor['state'] = DISABLED

    def send_clicked(self,_=None):
        send_str = self.ent_send.get()
        success = self.sensor_interface.send(send_str+"\n")
        # if(not success):
        #     self.txt_monitor['state'] = NORMAL
        #     self.txt_monitor.insert(1.0,"Failed to send\n")
        #     self.txt_monitor['state'] = DISABLED

        if(send_str != ""):
            if(send_str in self.send_cmd_list):
                self.send_cmd_list.remove(send_str)
            self.send_cmd_list.append(send_str)
            self.send_cmd_current = ""
            self.send_cmd_index = 1

            if(len(self.send_cmd_list)>CMD_LIST_MAX_SIZE):
                self.send_cmd_list.pop(0)

        self.updateUI()

    def selected_signals_changed(self,_):
        signals = self.ent_select.getArray()-1 # make 0-indexed
        print("DEBUG SensorGUI selected signals changed "+str(signals))
        if(signals.size > 0):
            self.sensor_interface.input_signal_sep = self.ent_select.seperator
            self.sensor_interface.set_selected_signals(signals)
        else:
            self.sensor_interface.input_signal_sep = ""
            self.sensor_interface.set_selected_signals(SensorInterface.SELECT_ALL_SIGNALS)

    def ent_send_up(self,_):

        # if(self.ent_send.get()==""): self.send_cmd_index=0

        # if(self.send_cmd_index==0 or (self.send_cmd_index==-1 and self.ent_send.get()!="")):
        #     self.send_cmd_current = self.ent_send.get()
        #     self.send_cmd_index=1

        if(self.send_cmd_index == -1 and self.ent_send.get() != ""
        or self.send_cmd_index ==  0 and self.ent_send.get() != self.send_cmd_current
        or self.send_cmd_index  >  0 and self.ent_send.get() != self.send_cmd_list[-self.send_cmd_index]): # field has been changed
            self.send_cmd_index=0
            self.send_cmd_current = self.ent_send.get()
            print("Changes were made!")
            print(self.send_cmd_index, end=": ")
            print(self.send_cmd_current) # debug

        self.send_cmd_index = min(self.send_cmd_index+1, len(self.send_cmd_list))

        if(self.send_cmd_index>0):
            gui.setText(self.ent_send, self.send_cmd_list[-self.send_cmd_index])
        elif(self.send_cmd_index==0):
            gui.setText(self.ent_send, self.send_cmd_current)

        print(self.send_cmd_index)

    def ent_send_down(self,_):
        if(self.send_cmd_index == -1 and self.ent_send.get() != ""
        or self.send_cmd_index ==  0 and self.ent_send.get() != self.send_cmd_current
        or self.send_cmd_index  >  0 and self.ent_send.get() != self.send_cmd_list[-self.send_cmd_index]): # field has been changed
            self.send_cmd_index=0
            self.send_cmd_current = self.ent_send.get()
            print("Changes were made!")
            print(self.send_cmd_index, end=": ")
            print(self.send_cmd_current) # debug

        if(self.send_cmd_index == 0 and self.ent_send.get()==""):
            self.send_cmd_index = max(self.send_cmd_index-1, 0)
        else:
            self.send_cmd_index = max(self.send_cmd_index-1, -1)

        if(self.send_cmd_index>0):
            gui.setText(self.ent_send, self.send_cmd_list[-self.send_cmd_index])
        elif(self.send_cmd_index==0):
            gui.setText(self.ent_send, self.send_cmd_current)
        else:
            gui.setText(self.ent_send, "")

        # if(self.ent_send.get()==""): self.send_cmd_index=0

        # if(self.send_cmd_index<=0):
        #     if(self.ent_send.get()!=""):
        #         self.send_cmd_current = self.ent_send.get()
        #         self.send_cmd_index = -1
        #         gui.setText(self.ent_send, "")
        # else:
        #     self.send_cmd_index = max(self.send_cmd_index-1, 0)

        #     if(self.send_cmd_index>0):
        #         gui.setText(self.ent_send, self.send_cmd_list[-self.send_cmd_index])
        #     elif(self.send_cmd_index==0):
        #         gui.setText(self.ent_send, self.send_cmd_current)

        print(self.send_cmd_index)


    def update_monitor(self):
        if self.sensor_interface.isConnected():
            scroll_pos = self.txt_monitor.yview()

            self.txt_monitor['state'] = NORMAL
            self.txt_monitor.delete(1.0, "end")
            self.txt_monitor.insert(1.0,self.sensor_interface.monitor_output)
            self.txt_monitor['state'] = DISABLED

            if(self.sensor_interface.signals_are_valid()):
                sepearor = self.sensor_interface.input_signal_sep
                comma_count = self.sensor_interface.last_line.count(sepearor)
                if(comma_count > 0):
                    str_l = str(self.sensor_interface.line_count)
                    cur_pos = 0
                    search_line = self.sensor_interface.last_line+sepearor
                    for i in range(comma_count+1):
                        idx = search_line.index(sepearor,cur_pos)
                        if(i in self.sensor_interface.selected_signals):
                            self.txt_monitor.tag_add("MEAS_VAL",str_l+"."+str(cur_pos), str_l+"."+str(idx))
                        cur_pos = idx+1

            if(scroll_pos[1]==1): # scrollbar at bottom
                self.txt_monitor.see('end')
            else:
                self.txt_monitor.yview_moveto(scroll_pos[0])

        self.after(MONITOR_REFRESH_DELAY, self.update_monitor)

    def checkNeedsUpdate(self):
        if(self.needs_ui_update): self.updateUI()
        self.after(100,self.checkNeedsUpdate)

    def updateUI(self):
        gui.set_enabled(self.btn_send,        self.sensor_interface.isConnected())
        if(not self.sensor_interface.isConnected()):
            self.port_sel.set_if_available(gui.PortSelector.ENTRY_DISCONNECT)

        gui.set_enabled(self.ent_select, self.sensor_interface.can_select_signals())

        self.needs_ui_update=False

    def initUI(self):
        frame = self

        frame.columnconfigure(0, weight=1)

        frame.rowconfigure(0, weight=0)
        frame.rowconfigure(1, weight=1)
        frame.rowconfigure(2, weight=0)

        monitor_head = ttk.Frame(frame)
        monitor_head.grid(row=0,column=0, padx=5, pady=5, sticky=S+E+W)

        monitor_body = ttk.Frame(frame)
        monitor_body.grid(row=1,column=0, padx=5, pady=2, sticky=N+S+E+W)

        monitor_foot = ttk.Frame(frame)
        monitor_foot.grid(row=2,column=0, padx=5, pady=5, sticky=N+S+E+W)

        # MONITOR HEADER
        monitor_head.columnconfigure(0,weight=1)
        self.port_sel = gui.PortSelector(monitor_head, "Port: ", entries=[gui.PortSelector.ENTRY_DISCONNECT], command=self.port_changed)
        self.port_sel.grid(row=0,column=0,sticky=(W,S,E),padx=(10,30))

        # MONITOR BODY
        monitor_body.columnconfigure(0, weight=1)
        monitor_body.rowconfigure(0, weight=1)

        self.def_font = Font(family="Helvetica", size=gui.DEFAULT_FONT_SIZE)

        self.txt_monitor = tk.Text(monitor_body, width=60,height=20, state=DISABLED,font=self.def_font)
        self.txt_monitor.grid(column=0, row=0, sticky=tk.NSEW)

        sb_monitor = ttk.Scrollbar(monitor_body,
                                    orient='vertical',
                                    command=self.txt_monitor.yview)

        sb_monitor.grid(column=1, row=0, sticky=tk.NS)
        self.txt_monitor['yscrollcommand'] = sb_monitor.set

        self.bold_font = Font(family="Helvetica", size=gui.DEFAULT_FONT_SIZE, weight="bold")
        self.txt_monitor.tag_configure("MEAS_VAL", font=self.bold_font)

        # MONITOR FOOTER
        monitor_foot.columnconfigure(0, weight=4)
        monitor_foot.columnconfigure(2, weight=2)
        monitor_foot.columnconfigure(3, weight=4)
        monitor_foot.columnconfigure(4, weight=2)

        self.lbl_select = tk.Label(monitor_foot, text="Select signals")
        self.lbl_select.grid(row=0,column=1,sticky=E,padx=(5,5))

        self.ent_select = gui.ListEntry(monitor_foot, width=20, allow_negative=False, val_type=int, on_edit=self.selected_signals_changed)
        self.ent_select.grid(row=0,column=2,sticky=W)

        self.ent_send = tk.Entry(monitor_foot, width=11)
        self.ent_send.insert(END, self.send_cmd_current)
        self.ent_send.grid(row=0,column=4,sticky=E+W,padx=(5,0))
        self.ent_send.bind('<Return>', self.send_clicked)
        self.ent_send.bind('<Up>', self.ent_send_up)
        self.ent_send.bind('<Down>', self.ent_send_down)
        # self.ent_send.bind('<KeyRelease>', self.ent_send_key)

        self.btn_send = Button(monitor_foot, text="Send", command=self.send_clicked, state=DISABLED)
        self.btn_send.grid(row=0,column=5,sticky=E+W,padx=(5,10))

    def save_UI_state(self, db):
        db["sensor_gui"] = {
            "ent_sel":self.ent_select.get(),
            "ent_send":self.ent_send.get(),
            "cmd_list":self.send_cmd_list,
            "sensor_port":self.port_sel.get()
        }
        return db

    def load_UI_state(self, db):
        d = db["sensor_gui"]
        if(d==None): return
        gui.setText(self.ent_select,d["ent_sel"])
        self.selected_signals_changed(None)
        # gui.setText(self.ent_send,d["ent_send"])
        self.send_cmd_list = d["cmd_list"]
        self.ent_send_up(None)
        self.port_sel.set_if_available(d["sensor_port"])

if __name__ == "__main__":
    window = tk.Tk()
    window.title("Sensor Monitor")
    window.style = ttk.Style()
    window.style.theme_use("alt")
    sensor = SensorInterface.SensorInterface()
    sf = SensorFrame(window, sensor)
    sf.pack(fill=BOTH, expand=True)

    window.mainloop()
    sensor.shutdown()
