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
            rospy.loginfo("Reopening serial port " +
                          self._serial_port + " for " + self._device_id)
        else:
            rospy.loginfo("Opening serial port " +
                          self._serial_port + " for " + self._device_id)
        self._ser = serial.Serial(self._serial_port, 57600, timeout=5)

    def serial_query(self, msg):
        self._ser.write(msg)
        self._ser.flush()
        read = self._readline()
        return read


class PwmDeviceServer:

    local_state = [0, 0, 0, 0, 0, 0]

    def __init__(self, device_id, serial_port_device):
        self._device_id = device_id
        self._serial_port_device = serial_port_device
        self._pwm_state_pub = rospy.Publisher(
            self._device_id + '/pwm', PwmState, queue_size=10)
        self.publish_state()

    def get_state(self):
        for i in range(3):  # Try 2 times to read the result if it didn't come the first time
            dataJson = self._serial_port_device.serial_query(
                json.dumps({'cmd': 'data'}, separators=(',', ':')))
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
                    return m
        self._serial_port_device.open()  # Reopen in case of error
        raise Exception("Unable to query the PWM device for its state.")

    def get_state_local(self):
        return self.local_state

    def publish_state(self):
        state = self.get_state_local()
        self._pwm_state_pub.publish(state)
        return state

    def handle_operate_pwm_device_request(self, req):
        try:
            dataJson = self._serial_port_device.serial_query(json.dumps(
                {'pwm_power_driver': {("ch_%d" % req.device): req.pwm}}, separators=(',', ':')))
            self.local_state[req.device] = req.pwm
#            if not dataJson:
#                raise Exception("Unable to get response from the PWM device from the last request.")
#            message = json.loads(dataJson)
#            if 'result' not in message and message['result'] != 'success':
#                raise Exception("Unable to get positive confirmation from PWM device from the last request.")
            state = self.publish_state()
            return OperatePwmDeviceResponse(state)
        except BaseException as e:
            rospy.logerr(
                "Exception occurred when reading from the PWM device: " + str(e))
            self._serial_port_device.open()

    def handle_operate_pwms_request(self, req):
        try:
            dataJson = self._serial_port_device.serial_query(json.dumps(
                {'pwm_power_driver': {
                    "ch_1": req.pwm[0],
                    "ch_2": req.pwm[1],
                    "ch_3": req.pwm[2],
                    "ch_4": req.pwm[3],
                    "ch_5": req.pwm[4],
                    "ch_6": req.pwm[5]}}, separators=(',', ':')))
            self.local_state = req.pwm
            if not dataJson:
                raise Exception(
                    "Unable to get response from the PWM devices from the last request.")
            message = json.loads(dataJson)
            if 'result' not in message and message['result'] != 'success':
                raise Exception(
                    "Unable to get positive confirmation from PWM devices from the last request.")
            self._pwm_state_pub.publish(req.pwm)
            return OperatePwmsResponse(req.pwm)
        except BaseException as e:
            rospy.logerr(
                "Exception occurred when reading from the PWM devices: " + str(e))
            self._serial_port_device.open()

    def handle_query_pwms_request(self, req):
        return QueryPwmsResponse(self.get_state_local())


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
        serviceLookup = rospy.ServiceProxy(
            "atmega328p_service_discovery/lookup", ServiceSerial)
        resp = serviceLookup(device_id)
        serial_port = resp.serial

        rospy.loginfo("Found serial port for service '{0}' on: {1}".format(
            device_id, serial_port))
        serial_port_device = SerialPortDevice(device_id, serial_port)
        serial_port_device.open()

        # Wait for the Arduino to come out of RESET mode (after DTR is pulled up
        # again)
        time.sleep(2)

        device_server = PwmDeviceServer(device_id, serial_port_device)

        pwm_state_pub = rospy.Publisher(
            device_id + '/pwm', PwmState, queue_size=10)
        pwm_service = rospy.Service(
            device_id + '/pwm', OperatePwmDevice, device_server.handle_operate_pwm_device_request)
        pwm_service = rospy.Service(
            device_id + '/pwms', OperatePwms, device_server.handle_operate_pwms_request)
        pwm_service = rospy.Service(
            device_id + '/query', QueryPwms, device_server.handle_query_pwms_request)
        rospy.spin()

    except rospy.ServiceException as exc:
        rospy.logerr(
            "The serial port service did not process request: " + str(exc))
        rospy.logerr("Shutting down node " + device_id)


if __name__ == "__main__":
    main()
