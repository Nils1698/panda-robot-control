import serial
import time


port_name = "/dev/ttyACM0"

 

BAUD = 500000

PORT_TIMEOUT = 0.5

PORT_WRITE_TIMEOUT=1

PORT_STATUS="PORT_NOT_OPEN"

last_line = ""
monitor_output = ""
 

serial_port = serial.Serial(port_name,BAUD,timeout=PORT_TIMEOUT,write_timeout=PORT_WRITE_TIMEOUT)
serial_port.reset_input_buffer()
PORT_STATUS="PORT_OPEN"

def send(string):
  print(f"Send: '{string}'")
  try:
    serial_port.write(string.encode())
    return True

  except serial.SerialTimeoutException:
    print("Send failed!")
  return False


send("init\n")

time.sleep(1)

send("sub-clear\n")

time.sleep(1)


while PORT_STATUS=="PORT_OPEN":
  i = 0
  x = input()
  send(f"{x}\n")

  if x == "close":
    serial_port.close()
    PORT_STATUS=="PORT_NOT_OPEN"

  while i < 200:
    print(last_line)
  #time.sleep(0.001) # yield thread
    try:
      serialString = serial_port.readline()
      if(serialString==b''): # time out
        continue

      try:
        last_line = serialString.decode("utf-8").replace("\r","").replace("\t"," ")
        #if(verbose): print("%  "+last_line)
        #handle_incoming(last_line)

      except UnicodeDecodeError:
        print("Decoding error: ", end='')
        print(serialString)
        continue

      monitor_output += last_line
      monitor_output = monitor_output.split("\n",1)[1] # remove first line

      i += 1

    except serial.SerialException:
      print("SensorInterface - Lost conenction!")
      PORT_STATUS = "PORT_CLOSING"
      break