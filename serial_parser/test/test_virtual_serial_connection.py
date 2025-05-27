"""
This script simulates a virtual serial connection by writing data to a serial port.
Before running this script, ensure that you have a virtual serial port set up.
The virtual port can be created using `socat` using the following command: 
    socat -d -d pty,raw,echo=0 pty,raw,echo=0
The above command will create two virtual serial ports, e.g., /dev/pts/2 and /dev/pts/3.
This script should publish to the first port, the serial_parser node should subscribe to the second port.
"""
import serial
import time

port = serial.Serial("/dev/pts/2", baudrate=115200)
rate = 1  # Hz
period = 1.0 / rate  # seconds

try:
    while True:
        print("Sending data to serial port...")
        # port.write(b"123.45\n\r")
        port.write(b"Hello, this is a test message!\n")
        time.sleep(period)  # simulate sending at 10 Hz
except KeyboardInterrupt:
    port.close()
