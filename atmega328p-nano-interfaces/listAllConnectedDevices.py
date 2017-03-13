#!/usr/bin/python

import sys
import json
import serial
import serial.tools.list_ports
import time


def get_device_id(serialport):
    print "### Found interface module on " + serialport
    ser = serial.Serial(serialport, 57600, timeout=0.5)
    time.sleep(2)
    ser.write(json.dumps({'cmd':'info'}))
    infoRawJson = ser.read(65536)
    if infoRawJson:
        infoMsg = json.loads(infoRawJson)
        print "    Module ID: " + infoMsg["id"]
        print "    Module Info: " + json.dumps(infoMsg)




def find_serial_port():
    print "Finding serial port interface modules (on ID 1a86:7523)"    
    print ""
    listPorts = serial.tools.list_ports.comports()

    for port in listPorts:
        if port.vid == 6790 and port.pid == 29987:
            get_device_id(port.device)
            print ""
    
    



def main():
    
                   
    serialport = find_serial_port()

    

if __name__ == "__main__": main()



    
    