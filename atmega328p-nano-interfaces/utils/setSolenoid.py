#!/usr/bin/python

import sys
import json
import serial
import serial.tools.list_ports
import time


polling_frequency = 2 # seconds

def get_device_id(serialport):
    ser = serial.Serial(serialport, 57600, timeout=0.5)
    time.sleep(2)
    ser.write(json.dumps({'cmd':'info'}))
    infoRawJson = ser.read(65536)
    if infoRawJson:
        infoMsg = json.loads(infoRawJson)
        return infoMsg["id"]
    
    return ""




def find_serial_port(id_string):
    print "Finding serial port interface module with ID " + id_string
    
    listPorts = serial.tools.list_ports.comports()

    for port in listPorts:
        if port.vid == 6790 and port.pid == 29987 and get_device_id(port.device) == id_string:
            return port.device
    
    raise Exception("Unable to find an interface module with the ID " + id_string)
    

def write(ser, channel, value):
    ser.write(json.dumps({'pwm_power_driver': {channel: value }}))
    dataJson = ser.read(65536)
    if dataJson:
        message = json.loads(dataJson)
        print("Data: ")
        print json.dumps(message, sort_keys=True, 
            indent=4, separators=(',', ': '))
        print ""


def main():
    

    ser = serial.Serial(sys.argv[1], 57600, timeout=0.5)

    while 1:
        time.sleep(2)
        write(ser,'ch_1',100)
        write(ser,'ch_2',100)
        write(ser,'ch_3',100)
        write(ser,'ch_4',100)
        write(ser,'ch_5',100)
        write(ser,'ch_6',100)
        time.sleep(2)
        write(ser,'ch_1',0)
        write(ser,'ch_2',0)
        write(ser,'ch_3',0)
        write(ser,'ch_4',0)
        write(ser,'ch_5',0)
        write(ser,'ch_6',0)

if __name__ == "__main__": main()



    
    
