#!/usr/bin/python

import sys
import json
import serial
import serial.tools.list_ports
import time
import rospy
from std_msgs.msg import Float64
from search_service import SearchService, LockHandle

polling_frequency = 1  # Hz


def main():
    if len(sys.argv) < 2:
        rospy.loginfo("Unable to start client with no identification string.")
        rospy.loginfo("Usage: client.py <identification string>")
        sys.exit(1)

    device_id = sys.argv[1]

    search = SearchService()

    serialport = search.find_serial_port(device_id)

    rospy.loginfo("Locking serial port " + serialport)

    ser = serial.Serial(serialport, 57600, timeout=0.5)
    lock = LockHandle(ser.fileno())
    lock.acquire()

    # Wait for the Arduino to come out of RESET mode (after DTR is pulled up
    # again)
    time.sleep(2)

    temperature_pub = rospy.Publisher(
        device_id + '/temperature', Float64, queue_size=10)
    humidity_pub = rospy.Publisher(
        device_id + '/humidity', Float64, queue_size=10)
    rospy.init_node(device_id, anonymous=True)
    rate = rospy.Rate(polling_frequency)

    while not rospy.is_shutdown():
        try:
            ser.write(json.dumps({'cmd': 'data'}))
            dataJson = ser.read(65536)
            if dataJson:
                message = json.loads(dataJson)
                if "sensors" in message and "si7021" in message['sensors']:
                    temperature_pub.publish(
                        message['sensors']['si7021']['temperature'])
                    humidity_pub.publish(message['sensors'][
                                         'si7021']['humidity'])
        except:
            print("Exception occurred when polling device.")
        rate.sleep()


if __name__ == "__main__":
    main()
