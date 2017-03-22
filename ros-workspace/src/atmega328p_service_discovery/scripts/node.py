#!/usr/bin/python
import sys
import json
import serial
import fcntl
import serial.tools.list_ports
import time
from threading import Thread, Lock
import rospy
from atmega328p_service_discovery.srv import *


class SearchServices:

    foundDevices = {}

    def get_serial_port(self, service):
        if not service in self.foundDevices:
            rospy.logwarn("Unable to find service '{0}' during service discovery.".format(service))
            return ""
        return self.foundDevices[service]

    def get_device_id(self, serialport):
        id = None
        for i in range(3):
            try:
                ser = serial.Serial(serialport, 57600, timeout=0.5)
                # Wait for the Arduino to come out of RESET
                # mode (after DTR is pulled up again)
                time.sleep(2)
                ser.write(json.dumps({'cmd': 'info'}))
                infoRawJson = ser.read(65536)
                if infoRawJson:
                    infoMsg = json.loads(infoRawJson)
                    id = infoMsg["id"]
                ser.close()
                break
            except IOError as e:
                rospy.logerror("IO error occurred when getting device ID ({0}): {1}".format(e.errno, e.strerror))
        if not id:
            rospy.logwarn("Unable to get device ID string from valid serial port device.")
        return id

    def discover_services(self):
        rospy.loginfo("Finding serial port interface modules.")
        listPorts = serial.tools.list_ports.comports()
  
        for port in listPorts:
            if port.vid == 6790 and port.pid == 29987:
                id = self.get_device_id(port.device)
                if id:
                    self.foundDevices[id] = port.device
                    rospy.loginfo("Found service '{0}' on device: {1}".format(id, port.device))

        if not self.foundDevices:
            rospy.logwarning("Unable to find any services on serial ports.")


def handle_server_request(req):
    global m_services
    return ServiceSerialResponse(m_services.get_serial_port(req.service))

m_services = SearchServices()


def main():

    rospy.init_node("atmega328p_service_discovery", anonymous=False)

    m_services.discover_services()

    pwm_service = rospy.Service(
        'atmega328p_service_discovery/lookup', ServiceSerial, handle_server_request)


    rospy.spin()


if __name__ == "__main__":
    main()

