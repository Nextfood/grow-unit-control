import sys
import json
import serial
import fcntl
import serial.tools.list_ports
import time
import rospy


class Lock:

    def __init__(self, filename):
        self.filename = filename
        # This will create it if it does not exist already
        self.handle = open(filename, 'w')

    def acquire(self):
        fcntl.flock(self.handle, fcntl.LOCK_EX)

    def acquireNB(self):
        fcntl.flock(self.handle, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def release(self):
        fcntl.flock(self.handle, fcntl.LOCK_UN)

    def __del__(self):
        self.handle.close()


class LockHandle:

    def __init__(self, handle):
        self.handle = handle

    def acquire(self):
        fcntl.flock(self.handle, fcntl.LOCK_EX)

    def acquireNB(self):
        fcntl.flock(self.handle, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def release(self):
        fcntl.flock(self.handle, fcntl.LOCK_UN)

class SearchService:

    def get_device_id(self, serialport):
        id = ""
        try:
            ser = serial.Serial(serialport, 57600, timeout=0.1)
            lock = LockHandle(ser.fileno())
            lock.acquireNB()
            # Wait for the Arduino to come out of RESET
            # mode (after DTR is pulled up again)
            time.sleep(2)
            ser.write(json.dumps({'cmd': 'info'}))
            infoRawJson = ser.read(65536)
            if infoRawJson:
                infoMsg = json.loads(infoRawJson)
                id = infoMsg["id"]
            ser.close()
        except IOError as e:
            rospy.loginfo("IO error occurred when getting device ID ({0}): {1}".format(e.errno, e.strerror))
        return id

    def find_serial_port(self, id_string):
        lock = Lock("/tmp/ros_serialport_scan.lock")
        lock.acquire()
        rospy.loginfo("Finding serial port interface module with ID '" + id_string + "'...")

        listPorts = serial.tools.list_ports.comports()

        foundDevice = None

        for port in listPorts:
            if port.vid == 6790 and port.pid == 29987 and self.get_device_id(port.device) == id_string:
                foundDevice = port.device
                break

        lock.release()
        if foundDevice:
            rospy.loginfo("Found matching serial port interface module on " + foundDevice)
            return foundDevice
        else:
            raise Exception(("Unable to find an interface module with the ID " + id_string))
