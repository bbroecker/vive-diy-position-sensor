#!/usr/bin/python
# usage: python cdc_reset.py <DEVICE>
# where <DEVICE> is typically some /dev/ttyXXX
import sys
import serial
import time

ser = serial.Serial()
ser.port = sys.argv[1]
ser.open()
ser.baudrate = 1200  # This is magic.
ser.rts = True
ser.dtr = False
ser.close()
time.sleep(1)
