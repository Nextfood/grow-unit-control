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

def serial_query(msg):
   serial_handle.write(msg)
   return serial_handle.read(65536)

def publish_relay_state():
    for i in range(3): # Try 2 times to read the result if it didn't come the first time
        dataJson = serial_query(json.dumps({'cmd': 'data'}))
        if dataJson:
            message = json.loads(dataJson)
            if "devices" in message and "relay" in message['devices']:
                state = message['devices']['relay']['state']
                relay_state_pub.publish(state)
                return state

def handle_server_request(req):
    dataJson = serial_query(json.dumps({'relay': {"state": req.state}}))
    state = publish_relay_state()
    return OperateRelayResponse(state)


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

    serial_handle = serial.Serial(serialport, 57600, timeout=0.1)
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
        device_id + '/relay', Bool, queue_size=10)
    relay_service = rospy.Service(
        device_id + '/relay', OperateRelay, handle_server_request)

    rospy.spin()


if __name__ == "__main__":
    main()
