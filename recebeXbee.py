####
## XBee: https://pypi.python.org/pypi/XBee/2.1.0
##
import serial, time, datetime, sys
from xbee import XBee,ZigBee

SERIALPORT = "/dev/ttyAMA0"    # the com/serial port the XBee is connected to, the pi GPIO should always be ttyAMA0
BAUDRATE = 9600      # the baud rate we talk to the xbee

ser = serial.Serial(SERIALPORT, BAUDRATE)

xbee = ZigBee(ser,escaped=True)

print 'Starting Up Tempature Monitor'
# Continuously read and print packets
while True:
    try:
        response = xbee.wait_read_frame()
        print response
    except KeyboardInterrupt:
        print 'fechei'
        break

ser.close()
