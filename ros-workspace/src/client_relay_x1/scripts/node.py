#!/usr/bin/python
import sys
import json
import serial
import serial.tools.list_ports
import time
from client_relay_x1.srv import *
import rospy
from std_msgs.msg import Bool
from atmega328p_service_discovery.srv import *

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

    try:
        publish_relay_state()
    except:
        rospy.logwarn("Exception occurred when reading from the relay device.")

    relay_state_pub = rospy.Publisher(
        device_id + '/relay', Bool, queue_size=10)
    relay_service = rospy.Service(
        device_id + '/relay', OperateRelay, handle_server_request)

    rospy.spin()


if __name__ == "__main__":
    main()
