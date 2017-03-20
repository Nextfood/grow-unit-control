#!/usr/bin/python

import sys
import json
import serial
import serial.tools.list_ports
import time


def main():

    ser = serial.Serial(sys.argv[1], 57600, timeout=0.5)
    time.sleep(2)

    ser.write(json.dumps({'relay': {'state': True}}))
    dataJson = ser.read(65536)
    if dataJson:
        message = json.loads(dataJson)
        print("Data: ")
        print json.dumps(message, sort_keys=True, 
            indent=4, separators=(',', ': '))
        print ""
    time.sleep(3)

    ser.write(json.dumps({'relay': {'state': False}}))
    dataJson = ser.read(65536)
    if dataJson:
        message = json.loads(dataJson)
        print("Data: ")
        print json.dumps(message, sort_keys=True, 
            indent=4, separators=(',', ': '))
        print ""

if __name__ == "__main__": main()



    
    
