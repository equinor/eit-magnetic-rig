import os
import serial

serial_port = os.environ.get("PORT", "/dev/pts/3")
serial_baudrate = os.environ.get("BAUDRATE", 9600)

com_port_instance = serial.Serial(serial_port, serial_baudrate)
com_port_instance.write(b'Hello')

# import serial
# import time

# # make sure to modify your ports
# PORT1 = '/dev/ttys008'
# PORT2 = '/dev/ttys009'

# ser1 = serial.Serial(PORT1, 115200)
# ser2 = serial.Serial(PORT2, 115200)

# while 1:
#     time.sleep(1)
  
#     # write to PORT1
#     message = b'hello world'
#     ser1.write(message)
#     print('writing: {}'.format(message))
  
#     time.sleep(1)
    
#     # read from PORT2
#     if ser2.inWaiting():
#         payload = ser2.read(ser2.inWaiting())
#         print('recv: {}'.format(payload))