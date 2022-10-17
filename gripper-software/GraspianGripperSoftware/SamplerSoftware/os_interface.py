
from tkinter import filedialog 
from tkinter import TclError
import pathlib
import tkinter
from serial.tools import list_ports
import os

def available_ports():
    all_port_tuples = list_ports.comports()
    ports = []
    for ap, desc, hwid in all_port_tuples:
        ports.append(ap)

    return ports

class OsInterface:
    def __init__(self, tk:tkinter.Tk, profile_folder=None):
        self.export_path = '~'
        self.export_name = ''
        # TODO keep track of conf file as well
        self.tk = tk
        self.profile_folder = profile_folder

        # Hide hidden files in save-dialog
        try:
            try:
                self.tk.call('tk_getOpenFile', '-foobarbaz')
            except TclError:
                pass

            self.tk.call('set', '::tk::dialog::file::showHiddenBtn', '1')
            self.tk.call('set', '::tk::dialog::file::showHiddenVar', '0')
        except:
            pass

        self.callbacks=[]

    def subscribe(self, cb):
        if(cb not in self.callbacks):
            self.callbacks.append(cb)

    def _callback(self):
        for cb in self.callbacks: cb()

    def update_path(self, fn):
        path = pathlib.Path(fn)
        self.export_path = path.parent
        self.export_name = path.stem
        self._callback()

    def save_dialog(self, ext, types, update_path:bool=True):
        EXT = "."+ext
        fn = filedialog.asksaveasfilename(defaultextension=EXT, initialdir = self.export_path, initialfile=self.export_name, filetypes=types)
        if(fn==""): return None

        if( pathlib.Path(fn).suffix != EXT): fn = fn+EXT
        if(update_path): self.update_path(fn)
        return fn

    def open_dialog(self, ext, types, update_path:bool=True):
        EXT = "."+ext
        fn = filedialog.askopenfilename(defaultextension=EXT, initialdir = self.export_path, initialfile=self.export_name, filetypes=types)
        if(fn==""): return None
        if(update_path): self.update_path(fn)
        return fn

    def save_png_dialog(self, update_path:bool=True):
        return self.save_dialog("png", (('PNG files', '*.png'),('All files', '*.*')), update_path)

    def save_csv_dialog(self, update_path:bool=True):
        return self.save_dialog("csv", (('CSV files', '*.csv'), ('Text files', '*.txt'),('All files', '*.*')), update_path)

    def load_csv_dialog(self, update_path:bool=True):
        return self.open_dialog("csv", (('CSV files', '*.csv'), ('Text files', '*.txt'),('All files', '*.*')), update_path)

    def save_conf_dialog(self):
        # fn = filedialog.askopenfilename(defaultextension=".conf", initialdir = self.profile_folder, filetypes=(('Configuration files', '*.conf')))
        fn = filedialog.asksaveasfilename(defaultextension=".conf", initialdir = self.profile_folder, filetypes=(('Configuration files', '*.conf'),('All files', '*.*')))
        if(fn==""): return None
        if( pathlib.Path(fn).suffix != ".conf"): fn = fn+".conf"
        return fn

    def save_UI_state(self, db):
        db["os_interface"] = {
            "export_path":self.export_path
        }
        return db

    def load_UI_state(self, db):
        d = db["os_interface"]
        if(d==None): return

        self.export_path = d["export_path"]

    def list_profiles(self) -> list:
        if(self.profile_folder==None): return []

        profiles = []
        for file in os.listdir(self.profile_folder):
            if file.endswith(".conf"):
                profiles.append((pathlib.Path(file).stem, os.path.join(self.profile_folder, file)))

        return profiles
