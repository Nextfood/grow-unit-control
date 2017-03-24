#!/usr/bin/python

import sys
import json
import serial
import serial.tools.list_ports
import time
import rospy
from std_msgs.msg import Float64
from atmega328p_service_discovery.srv import *

polling_frequency = 1  # Hz


def main():
    if len(sys.argv) < 2:
        rospy.loginfo("Unable to start client with no identification string.")
        rospy.loginfo("Usage: client.py <identification string>")
        sys.exit(1)

    device_id = sys.argv[1]

    rospy.init_node(device_id, anonymous=True)

    serial_port = None
    try:
        rospy.loginfo("Starting service client for '{0}'.".format(device_id))
        rospy.wait_for_service("atmega328p_service_discovery/lookup")
        serviceLookup = rospy.ServiceProxy("atmega328p_service_discovery/lookup", ServiceSerial)
        resp = serviceLookup(device_id)
        serial_port = resp.serial
    except rospy.ServiceException as exc:
        rospy.logerr("The serial port service did not process request: " + str(exc))


    rospy.loginfo("Found serial port for service '{0}' on: {1}".format(device_id, serial_port))

    serial_handle = serial.Serial(serial_port, 57600, timeout=0.1)

    # Wait for the Arduino to come out of RESET mode (after DTR is pulled up
    # again)
    time.sleep(2)

    light_data_full_pub = rospy.Publisher(
        device_id + '/full_spectrum', Float64, queue_size=10)
    light_data_ir_pub = rospy.Publisher(
        device_id + '/ir_spectrum', Float64, queue_size=10)
    light_data_lux_pub = rospy.Publisher(
        device_id + '/lux_visible_spectrum', Float64, queue_size=10)
    rate = rospy.Rate(polling_frequency)

    while not rospy.is_shutdown():
        try:
            ser.write(json.dumps({'cmd': 'data'}))
            dataJson = ser.read(65536)
            if dataJson:
                message = json.loads(dataJson)
                if "sensors" in message and "tsl2591" in message['sensors']:
                    light_data_full_pub.publish(
                        message['sensors']['tsl2591']['full'])
                    light_data_ir_pub.publish(
                        message['sensors']['tsl2591']['ir'])
                    light_data_lux_pub.publish(
                        message['sensors']['tsl2591']['lux'])
        except:
            print("Exception occurred when polling device.")
        rate.sleep()


if __name__ == "__main__":
    main()
