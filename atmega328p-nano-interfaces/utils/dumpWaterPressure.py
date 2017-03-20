#!/usr/bin/python

import sys
import json
import serial
import serial.tools.list_ports
import time

polling_frequency = 2 # seconds


def main():

    ser = serial.Serial("/dev/ttyUSB5", 57600, timeout=0.5)
    time.sleep(2)

    while True:
        ser.write(json.dumps({'cmd':'data'}))
        dataJson = ser.read(65536)
        if dataJson:
            message = json.loads(dataJson)
            print("Data: ")
            print json.dumps(message, sort_keys=True, 
                indent=4, separators=(',', ': '))
            print ""
        time.sleep(polling_frequency)

if __name__ == "__main__": main()
