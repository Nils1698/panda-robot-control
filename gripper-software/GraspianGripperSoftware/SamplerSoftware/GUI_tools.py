import tkinter as tk
from tkinter import *
from tkinter import ttk
from tkinter.font import Font
import numpy as np
from numpy.core.records import array
from os_interface import available_ports

DEFAULT_FONT_SIZE = 11

def set_enabled(widget:Widget, b):
    if(b): widget['state']=NORMAL
    else: widget['state']=DISABLED

def setText(entry:Entry, text:str):
    entry.delete(0,END)
    entry.insert(0,text)

class NumberEntry(Entry):
    def __init__(self, master, value=None, allow_negative=True, decimals=-1, value_type=float, max_val=float('inf'), min_val=float('-inf'), step_size=-1, on_edit=None, textvariable=None, *args, **kwargs):
        if(textvariable==None): self.sv = StringVar()
        else: self.sv = textvariable
        
        super(NumberEntry, self).__init__(master, textvariable=self.sv, *args, **kwargs)
        self.config( validate = 'key', validatecommand = (self.register(self.validateNumber), '%P'))
        self.bind("<FocusOut>", self.on_focus_out)

        self.allow_negative = allow_negative
        self.decimals = decimals
        self.max_val=max_val
        self.min_val=min_val
        self.step_size=step_size
        self.on_edit=on_edit
        self.value=None
        self.value_type=value_type
        self._ignore_value_change=False
        if(value): self.setValue(value)
        
    def validateNumber(self, value_if_allowed):
        try:
            new_value = self.value_type(value_if_allowed)
            valid = self.allow_negative or new_value>=0
        except ValueError:
            new_value = None
            valid = False

        valid = valid or value_if_allowed=="" or (self.allow_negative and value_if_allowed=="-")

        if(valid and self.value!=new_value):
            self.value = new_value
            if(self.value!=None):
                self.value = min(self.max_val, max(self.min_val, self.value ))
                if(self.step_size>0):
                    self.value = round(self.value/self.step_size)*self.step_size

            if(self.on_edit!=None and not self._ignore_value_change):
                self.on_edit(self)

        return valid

    def isValid(self)->bool:
        return self.value!=None

    def on_focus_out(self,_):
        if(self.isValid()):
            if(self.decimals >= 0):
                new_str = f"{self.value:.{self.decimals}f}"
            else:
                new_str = str(self.value)
                
            self.sv.set(new_str)

    def getValue(self):
        if(self.isValid()):
            return self.value
        else:
            return 0

    def setValue(self,value, dont_callback=False):
        self._ignore_value_change=dont_callback
        self.sv.set(value)
        self.on_focus_out(None)
        self._ignore_value_change=False

class ListEntry(Entry):
    def __init__(self, master, seperator=",", val_type=int, allow_negative=False, allow_zero=True, on_edit=None, textvariable=None, *args, **kwargs):
        if(textvariable==None): self.sv = StringVar()
        else: self.sv = textvariable

        super(ListEntry, self).__init__(master, textvariable=self.sv, *args, **kwargs)
        self.config( validate = 'key', validatecommand = (self.register(self.validate), '%P'))

        self.val_type=val_type
        self.on_edit=on_edit
        self.allow_neggative=allow_negative
        self.allow_zero=allow_zero
        self.value = np.array([])
        self.seperator = ','

    def _convert(self,entry_str):
        if(',' in entry_str): self.seperator=','
        else:                 self.seperator=' '
        
        arr = entry_str.split(self.seperator)
        no_empty = list(filter(lambda x: x!="" and (not self.allow_neggative or x!="-"),arr))

        if(self.val_type==int and (':' in entry_str) ):
            arr = np.array([], dtype=self.val_type)
            for e in no_empty:
                if(':' in e):
                    from_to = e.split(':')
                    if(len(from_to) != 2): raise ValueError
                    from_to = list(filter(lambda x: x!="", from_to)) # remove empty before converting (to allow 'x:' or ':x')
                    from_to = np.array(from_to, dtype=int)
                    if(from_to.size==2):
                        arr = np.append(arr, np.arange(from_to[0],from_to[1]+1))
                    elif(from_to.size==1):
                        arr = np.append(arr, int(from_to[0]))
                else:
                    pass
                    arr = np.append(arr, int(e))
                print("ListEntry - "+str(arr))
        else:
            arr = np.array(no_empty,dtype=self.val_type)

        if(self.allow_neggative==False and np.any(arr<0)): raise ValueError
        if(self.allow_zero==False      and np.any(arr==0)): raise ValueError
        return arr

    def validate(self, value_if_allowed):
        try:
            self.value = self._convert(value_if_allowed)
            if(self.on_edit!=None and self.get() != value_if_allowed): self.on_edit(self)
            return True
        except ValueError:
            print("DEBUG ListEntry validate failed")
            # print(value_if_allowed)
            return False

    def getArray(self) -> np.ndarray:
        return self.value

class PortSelector(ttk.Frame):
    ENTRY_DISCONNECT = "Disconnect"

    def __init__(self, parent, label, command, entries=[]):
        super().__init__(parent)
        self.callback = command
        self.entries = entries

        Label(self,text = label, width=6).pack(side=LEFT)

        self.cmb = ttk.Combobox(self,values=[])
        self.cmb.pack(side=LEFT, fill=X, expand=True)

        self.cmb.bind('<<ComboboxSelected>>', self.combo_selected) 
        self.cmb.bind("<Button-1>",           self.combo_clicked)

    def combo_clicked(self, w):
        self.cmb['values'] = available_ports() + self.entries

    def combo_selected(self, _):
        self.callback(self.cmb.get())

    def set_if_available(self, entry):
        if(entry == self.get()): return

        self.combo_clicked(None)
        try:
           self.cmb.current(self.cmb['values'].index(entry))
           self.callback(self.cmb.get())
        except ValueError:
            return

    def get(self):
        return self.cmb.get()

class MultiItemDropdown(ttk.Combobox):

    SEL_ALL_STR = "Select all"
    DESEL_ALL_STR = "Deselect all"

    def __init__(self, master, items:list=[], include_sel_all=True, on_open=None, on_select=None, *args, **kwargs):
        super().__init__(master, state="readonly", *args, **kwargs)
        self.include_sel_all = include_sel_all
        self.on_open = on_open
        self.on_select = on_select

        self.bind('<<ComboboxSelected>>', self.combo_selected) 
        self.bind("<Button-1>",           self.combo_clicked)

        self.selected_indices = []
        self.items = items

    def combo_clicked(self, w): # triggered when menu will show
        if(self.on_open!=None): self.on_open(self)
        vals = []
        for i in range(len(self.items)):
            str_val = ""
            if(i in self.selected_indices):
                str_val += "*"
            str_val += self.items[i]
            vals.append(str_val)

        if(self.include_sel_all):
            if(len(self.selected_indices)==len(self.items)): vals.append(self.DESEL_ALL_STR)
            else:                                            vals.append(self.SEL_ALL_STR)
        self['values'] = vals

    def combo_selected(self, _): # triggered when item selected
        if(self.get()==self.SEL_ALL_STR):
            self.selected_indices = list(range(len(self.items)))
        elif(self.get()==self.DESEL_ALL_STR):
            self.selected_indices = []
        elif(self.current() in self.selected_indices):
            self.selected_indices.remove(self.current())
        else:
            self.selected_indices.append(self.current())

        self.updateUI()
        if(self.on_select!=None): self.on_select(self)

    def updateUI(self):
        self.selected_indices.sort()
        print(f"selected indices: {self.selected_indices}")

        print(self.items)
        print(", ".join(self.items))
        self["values"] = [", ".join(self.getSelectedItems())]
        self.current(0)

    def getItems(self):
        return self.items.copy()

    def setItems(self, new_items:list, clear_selection=False):
        if(clear_selection): self.selected_indices = []
        else:
            new_selection = []
            for sel_item in self.getSelectedItems():
                if(sel_item in new_items):
                    new_selection.append(new_items.index(sel_item))
            
            self.selected_indices = new_selection

        self.items = new_items
        self.updateUI()

    def getSelectedIndices(self) -> list:
        return self.selected_indices.copy()

    def setSelectedIndices(self, sel:list):
        sel = list(filter(lambda i: i>=0 and i < len(self.items), sel))
        self.selected_indices = sel.copy()
        self["values"] = [", ".join(self.getSelectedItems())]
        self.current(0)

    def getSelectedItems(self) -> list:
        return [self.items[i] for i in self.selected_indices]

class ToggleButton(Button):
    STATE_ON = True
    STATE_OFF = False
    def __init__(self, master, text_on="", text_off=None, init_state=STATE_OFF, *args, **kwargs):
        if(text_off==None): text_off=text_on
        self.text_on  = text_on
        self.text_off = text_off
        self.state = init_state
        if("command" in kwargs):  self.super_command = kwargs["command"]
        else:                     self.super_command = None
        kwargs["command"] = self._on_click
        super().__init__(master, *args, **kwargs)
        self._update()

    def _update(self):
        if(self.state):
            self.config(relief="sunken", text=self.text_on)
        else:
            self.config(relief="raised", text=self.text_off)

    def _on_click(self):
        self.state = not self.state
        self._update()
        if(self.super_command): self.super_command()

    def getState(self):
        return self.state

    def setState(self, state):
        self.state = state
        self._update()

    def isOn(self):
        return self.state