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

class SerialPortDevice:

    _ser = None

    def __init__(self, device_id, serial_port):
        self._device_id = device_id
        self._serial_port = serial_port

    def _readline(self):
        eol = b'\n'
        leneol = len(eol)
        line = bytearray()
        while True:
            c = self._ser.read(1)
            if c:
                line += c
                if line[-leneol:] == eol:
                    break
            else:
                break
        return bytes(line)

    def open(self):
        if self._ser:
            self._ser.close()
            rospy.loginfo("Reopening serial port "+self._serial_port+" for "+self._device_id)
        else:
            rospy.loginfo("Opening serial port "+self._serial_port+" for "+self._device_id)
        self._ser = serial.Serial(self._serial_port, 57600, timeout=5)

    def serial_query(self, msg):
        self._ser.write(msg)
        self._ser.flush()
        read = self._readline()
        return read

class RelayDeviceServer:

    def __init__(self, device_id, serial_port_device):
        self._device_id = device_id
        self._serial_port_device = serial_port_device
        self._relay_state_pub = rospy.Publisher(
            self._device_id + '/relay', Bool, queue_size=10)
        self.publish_relay_state()

    def get_state(self):
        for i in range(3): # Try 2 times to read the result if it didn't come the first time
            dataJson = self._serial_port_device.serial_query(json.dumps({'cmd': 'data'}, separators=(',',':')))
            if dataJson:
                message = json.loads(dataJson)
                if "devices" in message and "relay" in message['devices']:
                    state = message['devices']['relay']['state']
                    return state
        self._serial_port_device.open() # Reopen in case of error
        raise Exception("Unable to query the relay device for its state.")

    def publish_relay_state(self):
        state = self.get_state()
        self._relay_state_pub.publish(state)
        return state

    def handle_operate_server_request(self, req):
        try:
            dataJson = self._serial_port_device.serial_query(json.dumps({'relay': {"state": req.state}}, separators=(',',':')))

            if not dataJson:
                raise Exception("Unable to get response from the relay device from the last request.")
            message = json.loads(dataJson)
            if 'result' not in message and message['result'] != 'success':
                raise Exception("Unable to get positive confirmation from relay device from the last request.")
            state = self.publish_relay_state()
            return OperateRelayResponse(state)
        except BaseException as e:
            rospy.logerr("Exception occurred when reading from the relay device: " + str(e))
            self._serial_port_device.open()

    def handle_state_server_request(self, req):
        try:
            state = self.get_state()
            return StateRelayResponse(state)
        except BaseException as e:
            rospy.logerr("Exception occurred when getting state from the relay device: " + str(e))
            self._serial_port_device.open()


def main():
    if len(sys.argv) < 2:
        print("Unable to start client with no identification string.")
        print("Usage: client.py <identification string>")
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

        rospy.loginfo("Found serial port for service '{0}' on: {1}".format(device_id, serial_port))
        serial_port_device = SerialPortDevice(device_id, serial_port)
        serial_port_device.open()

        # Wait for the Arduino to come out of RESET mode (after DTR is pulled up
        # again)
        time.sleep(2)

        relay_device_server = RelayDeviceServer(device_id, serial_port_device)

        relay_service = rospy.Service(
            device_id + '/relay', OperateRelay, relay_device_server.handle_operate_server_request)
        relay_service = rospy.Service(
            device_id + '/state', StateRelay, relay_device_server.handle_state_server_request)

        rospy.spin()

    except rospy.ServiceException as exc:
        rospy.logerr("The serial port service did not process request: " + str(exc))
        rospy.logerr("Shutting down node "+device_id)


if __name__ == "__main__":
    main()
