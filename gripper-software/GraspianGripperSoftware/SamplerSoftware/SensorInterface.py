from threading import Thread, Lock

import serial
import time

import numpy as np

#BAUD = 115200
BAUD = 500000
MONITOR_MAX_LINES = 200
PORT_TIMEOUT = 0.5
PORT_WRITE_TIMEOUT=1

# LOGGER_SIZE = 500

PORT_NOT_OPEN = 0
PORT_OPEN = 1
PORT_CLOSING = 2

DEFAULT_SINGAL_SEP = " "

SELECT_ALL_SIGNALS = np.array([])


CB_CONNECTION_CLOSED = "Connection Closed"
CB_CONNECTION_OPENED = "Connection Open"
CB_SIGNAL_SELECTION_CHANGED = "Selected signals changed"
CB_SIGNALS_CHANGED = "Signals changed"

class SensorInterface(Thread):

    def __init__(self):
        super().__init__()
        self.verbose = True
        self.serial_port = None
        self.callbacks = []

        self.port_status = PORT_NOT_OPEN
        self.expected_signal_size = 0

        self.reset_output()

        self.input_signal_sep=DEFAULT_SINGAL_SEP

        self._running = True
        self.start()

    def reset_output(self):
        self.last_line = ""
        self.monitor_output = ""
        self.line_count = 0

        self._current_signals = np.array([])
        self.selected_signals:np.ndarray = SELECT_ALL_SIGNALS
        # self.expected_signal_size = 0
        self.signal_selection_locked=False
        self._current_signal_id:int = 0
        self._signal_lock = Lock()

    def subscribe(self, cb):
        if(cb not in self.callbacks):
            self.callbacks.append(cb)

    def _callback(self, event):
        for cb in self.callbacks:
            cb(event)

    def handle_incoming(self, last_line):
        with self._signal_lock:
            new_signals = self._split_signals(last_line, self.selected_signals)
            if(new_signals.size != self.expected_signal_size):
                if(self._all_signals_selected() and self.signal_selection_locked==False):
                    print(f"SensorInterface - updating expected number of signals {self.expected_signal_size}->{new_signals.size}") # debug
                    self.expected_signal_size = new_signals.size
                    self._callback(CB_SIGNALS_CHANGED)
                # debug TODO delete
                # else:
                #     print(f"SensorInterface - warning: number of signals {new_signals.size} doesn't match expectation {self.expected_signal_size} (ignoring)")
                #     print(f"selected_signals: {self.selected_signals}, signal_selection_locked: {self.signal_selection_locked}")

            if(new_signals.size == self.expected_signal_size):
                self._current_signals = new_signals
                self._current_signal_id += 1

    def run(self):
        i=0 # debug
        print("Sensor thread running")
        while self._running:
            if(self.port_status==PORT_CLOSING):
                if(self.serial_port != None): self.serial_port.close()
                self.serial_port = None
                self.port_status = PORT_NOT_OPEN
                self._callback(CB_CONNECTION_CLOSED)
                self.reset_output()
                print("sensor port closed")
            
            while self.port_status==PORT_OPEN:
                time.sleep(0.001) # yield thread
                try:
                    serialString = self.serial_port.readline()
                    if(serialString==b''): # time out
                        continue

                    try:
                        self.last_line = serialString.decode("utf-8").replace("\r","").replace("\t"," ")
                        if(self.verbose): print("%  "+self.last_line)
                        self.handle_incoming(self.last_line)

                    except UnicodeDecodeError:
                        print("Decoding error: ", end='')
                        print(serialString)
                        continue

                    self.monitor_output += self.last_line
                    if(self.line_count >= MONITOR_MAX_LINES):
                        self.monitor_output = self.monitor_output.split("\n",1)[1] # remove first line
                    else:
                        self.line_count+=1

                except serial.SerialException:
                    print("SensorInterface - Lost conenction!")
                    self.port_status = PORT_CLOSING
                    break

                # debug
                i+=1
                if(self.verbose and i % 50 == 0):
                    print("Sensor loop running")

                # [PORT_OPEN loop] #

            time.sleep(0.1) # yield thread

            # [_running loop] #

        print("Sensor loop ended")

    def send(self,string):
        print(f"Send: '{string}'")
        if(self.isConnected()):
            try:
                self.serial_port.write(string.encode())
                return True
            except serial.SerialTimeoutException:
                print("Send failed!")
        return False

    def connect(self, port_name):
        if(self.isConnected()):
            self.disconnect()

        try:
            self.serial_port = serial.Serial(port_name,BAUD,timeout=PORT_TIMEOUT,write_timeout=PORT_WRITE_TIMEOUT)
            self.serial_port.reset_input_buffer()
            self.port_status = PORT_OPEN
            self._callback(CB_CONNECTION_OPENED)
            print("sensor connected")
            self.send("init\n")
            return True

        except OSError as e:
            print("Nothing found at '{}'".format(port_name))
            return False
        except serial.SerialTimeoutException:
            print("Failed to open port!")
            return False


    def disconnect(self):
        if(self.isConnected()):
            print("Close sensor port .. ")
            self.port_status = PORT_CLOSING
            i=0
            while(self.port_status != PORT_NOT_OPEN):
                time.sleep(0.01) # wait for monitor loop to end
                if(i>1/0.01):
                    raise Exception("Timed out closing sensor port!")
                    return False
                i+=1
        else:
            return False

        return True

    def isConnected(self):
        return self.port_status==PORT_OPEN

    def can_select_signals(self):
        return not self.signal_selection_locked

    def set_selected_signals(self, selection:np.ndarray):
        if(self.signal_selection_locked): return False

        if(not np.array_equal(self.selected_signals, selection)):
            self.selected_signals = selection[:]
            self._current_signals = self._split_signals(self.last_line, self.selected_signals)
            if(self._all_signals_selected()):
                self.expected_signal_size = self._current_signals.size
            else:
                self.expected_signal_size = selection.size

            self._callback(CB_SIGNAL_SELECTION_CHANGED)

        return True

    def _all_signals_selected(self):
        return self.selected_signals.size==0

    def signals_are_valid(self):
        return self._current_signals.size > 0 and self._current_signals.size==self.expected_signal_size

    def _split_signals(self, sample:str, selection) -> np.ndarray:
        if(self.input_signal_sep==""):
            if(sample.count(",")>0): self.input_signal_sep=","
            else: self.input_signal_sep=" "

        try:
            sample = sample.replace("\n","").rstrip(self.input_signal_sep)
            signals = np.array(sample.split(self.input_signal_sep)).astype(np.float)
        except:
            return np.array([])
        
        if(selection.size==0): return signals
        valid_selection = selection[(selection>=0) & (selection < signals.size)] # remove invalid indices
        return signals[valid_selection]

    def lock_signals(self):
        with self._signal_lock:
            self.signal_selection_locked=True

        # if(self._all_signals_selected()):
        #     self.selected_signals = np.arange(self._current_signals.size)

        self._callback(CB_SIGNAL_SELECTION_CHANGED)

    def unlock_signals(self):
        with self._signal_lock:
            self.signal_selection_locked=False
        self._callback(CB_SIGNAL_SELECTION_CHANGED)

    def current_signals(self) -> np.ndarray:
        with self._signal_lock:
            return self._current_signals.copy()

    def current_signal_id(self):
        with self._signal_lock:
            return self._current_signal_id
        
    def number_of_signals(self):
        return self.expected_signal_size

    def shutdown(self):
        self.disconnect()
        self._running = False