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
for i in range(10):
  send("grip 2\n")
  time.sleep(1)
  send("ctrl stop\n")
  time.sleep(2)
  send("m pos\n")
  time.sleep(0.5)
  send("ctrl stop\n")
  time.sleep(0.5)
  send("m open\n")
  time.sleep(2)
'''
  for i in range(20):
    try:
      serialString = serial_port.readline()
      if(serialString==b''): # time out
        continue

      try:
        last_line = serialString.decode("utf-8").replace("\r","").replace("\t"," ")

      except UnicodeDecodeError:
        print("Decoding error: ", end='')
        print(serialString)
        continue


    except serial.SerialException:
      print("SensorInterface - Lost conenction!")
      PORT_STATUS = "PORT_CLOSING"
      break

    measurements = last_line.replace("\n","").split(" ")[:-3]
    if last_line[0:5] != "debug":
      print(last_line[0:5])'''
#while PORT_STATUS=="PORT_OPEN":
#  i = 0
#  x = input()
#  send(f"{x}\n")
#  time.sleep(1)

#  if x == "close":
#    serial_port.close()
  #    PORT_STATUS=="PORT_NOT_O