#!/usr/bin/python

import sys
import json
import serial
import serial.tools.list_ports
import time

def set_serialport_id(serialport, idstring):
    print "Opening interface module on " + serialport
    ser = serial.Serial(serialport, 57600, timeout=0.5)
    time.sleep(2)
    ser.write(json.dumps({'id':idstring}))
    responseJson = ser.read(65536)
    if responseJson:
        infoMsg = json.loads(responseJson)
        print "Response: " + json.dumps(infoMsg)


def main():
    
    if len(sys.argv) < 3:
        print "Unable to start client with no serial port or identification string."
        print "Usage: client.py <serialport> <ID string>"
        sys.exit(1)

    set_serialport_id(sys.argv[1], sys.argv[2])
      
    

if __name__ == "__main__": main()



    
    