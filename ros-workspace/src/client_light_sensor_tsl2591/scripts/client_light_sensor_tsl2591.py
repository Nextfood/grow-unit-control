#!/usr/bin/python

import sys
import json
import serial
import fcntl
import serial.tools.list_ports
import time
import rospy
from std_msgs.msg import String,Float64

polling_frequency = 1 # Hz

class Lock:
    
    def __init__(self, filename):
        self.filename = filename
        # This will create it if it does not exist already
        self.handle = open(filename, 'w')
    
    # Bitwise OR fcntl.LOCK_NB if you need a non-blocking lock 
    def acquire(self):
        fcntl.flock(self.handle, fcntl.LOCK_EX)
        
    def release(self):
        fcntl.flock(self.handle, fcntl.LOCK_UN)
        
    def __del__(self):
        self.handle.close()


def get_device_id(serialport):
    id = ""
    ser = serial.Serial(serialport, 57600, timeout=0.5)
    fcntl.flock(ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    time.sleep(2) # Wait for the Arduino to come out of RESET mode (after DTR is pulled up again)
    ser.write(json.dumps({'cmd':'info'}))
    infoRawJson = ser.read(65536)
    if infoRawJson:
        infoMsg = json.loads(infoRawJson)
        id = infoMsg["id"]
    ser.close()
    return id




def find_serial_port(id_string):
    lock = Lock("/tmp/ros_serialport_scan.lock")
    lock.acquire()
    rospy.loginfo("Finding serial port interface module with ID '" + id_string + "'...")
    
    listPorts = serial.tools.list_ports.comports()

    foundDevice = None

    for port in listPorts:
        if port.vid == 6790 and port.pid == 29987 and get_device_id(port.device) == id_string:
            foundDevice = port.device
            break
    
    lock.release()
    if foundDevice:
        rospy.loginfo("Found matching serial port interface module on " + foundDevice)
        return foundDevice
    else:
        raise Exception(("Unable to find an interface module with the ID " + id_string))
    



def main():
    
    if len(sys.argv) < 2:
        rospy.loginfo("Unable to start client with no identification string.")
        rospy.loginfo("Usage: client.py <identification string>")
        sys.exit(1)
          
    device_id = sys.argv[1]
          
    serialport = find_serial_port(device_id)

    ser = serial.Serial(serialport, 57600, timeout=0.5)
    # Lock the serial port so other processes don't try to open it
    fcntl.flock(ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    time.sleep(2) # Wait for the Arduino to come out of RESET mode (after DTR is pulled up again)

    light_data_full_pub = rospy.Publisher(device_id+'/full_spectrum', Float64, queue_size=10)
    light_data_ir_pub = rospy.Publisher(device_id+'/ir_spectrum', Float64, queue_size=10)
    light_data_lux_pub = rospy.Publisher(device_id+'/lux_visible_spectrum', Float64, queue_size=10)
    rospy.init_node(device_id, anonymous=True)
    rate = rospy.Rate(polling_frequency)

    while not rospy.is_shutdown():
        ser.write(json.dumps({'cmd':'data'}))
        dataJson = ser.read(65536)
        if dataJson:
            message = json.loads(dataJson)
            if "sensors" in message and "tsl2591" in message['sensors']:
		        light_data_full_pub.publish(message['sensors']['tsl2591']['full'])
        		light_data_ir_pub.publish(message['sensors']['tsl2591']['ir'])
		        light_data_lux_pub.publish(message['sensors']['tsl2591']['lux'])
        rate.sleep()
    

if __name__ == "__main__": main()



    
    