#!/usr/bin/python
import sys
import json
import serial
import serial.tools.list_ports
import time
import rospy
from client_pwm_driver_x6.msg import *
from client_pwm_driver_x6.srv import *
from atmega328p_service_discovery.srv import *

pwm_state_pub = None
serial_handle = None

def serial_query(msg):
   serial_handle.write(msg)
   return serial_handle.read(65536)

def publish_pwm_state():
    for i in range(3): # Try 2 times to read the result if it didn't come the first time
        dataJson = serial_query(json.dumps({'cmd': 'data'}))
        if dataJson:
            message = json.loads(dataJson)
            if "devices" in message and "pwm_power_driver" in message['devices']:
                m = [None] * 6
                m[0] = message['devices']['pwm_power_driver']['ch_1']
                m[1] = message['devices']['pwm_power_driver']['ch_2']
                m[2] = message['devices']['pwm_power_driver']['ch_3']
                m[3] = message['devices']['pwm_power_driver']['ch_4']
                m[4] = message['devices']['pwm_power_driver']['ch_5']
                m[5] = message['devices']['pwm_power_driver']['ch_6']
                pwm_state_pub.publish(PwmState(m))
                return m


def handle_server_request(req):
    dataJson = serial_query(json.dumps({'pwm_power_driver': {("ch_%d"%req.device): req.pwm}}))
    state = publish_pwm_state()
    return OperatePwmResponse(state)


def main():
    global pwm_state_pub, serial_handle
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
        publish_pwm_state()
    except:
        rospy.logwarn("Exception occurred when reading from the PWM device.")


    pwm_state_pub = rospy.Publisher(
        device_id + '/pwm', PwmState, queue_size=10)
    pwm_service = rospy.Service(
        device_id + '/pwm', OperatePwm, handle_server_request)

    rospy.spin()


if __name__ == "__main__":
    main()
