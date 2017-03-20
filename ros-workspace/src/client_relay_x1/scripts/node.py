#!/usr/bin/python

import sys
import json
import serial
import serial.tools.list_ports
import time
from client_relay_x1.srv import *
import rospy
from std_msgs.msg import Bool
from search_service import SearchService, LockHandle

relay_state_pub = None
serial_handle = None


def publish_relay_state():
    serial_handle.write(json.dumps({'cmd': 'data'}))
    dataJson = serial_handle.read(65536)
    if dataJson:
        message = json.loads(dataJson)
        if "device" in message and "relay" in message['device']:
            state = message['device']['relay']['state']
            relay_state_pub.publish(state)


def handle_server_request(req):
    serial_handle.write(json.dumps({'relay': {'state': req.state}}))
    try:
        state = publish_relay_state()
        return OperateRelayResponse(state)
    except:
        raise ServiceException("Exception occurred when reading relay state.")


def main():
    global relay_state_pub, serial_handle
    if len(sys.argv) < 2:
        rospy.loginfo("Unable to start client with no identification string.")
        rospy.loginfo("Usage: client.py <identification string>")
        sys.exit(1)

    device_id = sys.argv[1]

    search = SearchService()

    serialport = search.find_serial_port(device_id)

    rospy.loginfo("Locking serial port " + serialport)

    serial_handle = serial.Serial(serialport, 57600, timeout=0.5)
    lock = LockHandle(serial_handle.fileno())
    lock.acquire()

    # Wait for the Arduino to come out of RESET mode (after DTR is pulled up
    # again)
    time.sleep(2)

    try:
        publish_relay_state()
    except:
        rospy.logwarn("Exception occurred when reading from the relay device.")

    rospy.init_node(device_id, anonymous=True)

    relay_state_pub = rospy.Publisher(
        device_id + '/relay_on', Bool, queue_size=10)
    relay_service = rospy.Service(
        device_id + '/relay', OperateRelay, handle_server_request)

    rospy.spin()


if __name__ == "__main__":
    main()
