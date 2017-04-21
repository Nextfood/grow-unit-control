#!/usr/bin/python

import sys
import json
import serial
import serial.tools.list_ports
import time


polling_frequency = 2  # seconds


class SerialPortDevice:

    _ser = None

    def __init__(self, serial_port):
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
        self._ser = serial.Serial(self._serial_port, 57600, timeout=5)

    def serial_query(self, msg):
        self._ser.write(msg)
        self._ser.flush()
        read = self._readline()
        return read


def main():

    if len(sys.argv) < 2:
        print "Unable to start client with no serial device."
        print "Usage: client.py <serial device>"
        sys.exit(1)

    serial_port_device = SerialPortDevice(sys.argv[1])
    serial_port_device.open()
    time.sleep(2)

    print "Connected."
    while True:
        dataJson = serial_port_device.serial_query(json.dumps({'cmd': 'info'})+"\n")
        if dataJson:
            print "Info: " + dataJson
#            message = json.loads(dataJson)
            print ""
   #     time.sleep(polling_frequency)


if __name__ == "__main__":
    main()
