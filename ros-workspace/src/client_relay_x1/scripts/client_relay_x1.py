import sys
import json
import serial
import serial.tools.list_ports
import time

identification_string = sys.argv
polling_frequency = 2 # seconds

def get_device_id(serialport):
    print "Found interface module on " + serialport
    ser = serial.Serial(serialport, 57600, timeout=0.5)
    time.sleep(2)
    ser.write(json.dumps({'cmd':'info'}))
    infoRawJson = ser.read(65536)
    if infoRawJson:
        infoMsg = json.loads(infoRawJson)
        print "Module Info: " + json.dumps(infoMsg)
        return infoMsg["id"]
    
    return ""




def find_serial_port(id_string):
    print "Finding serial port interface module with ID " + id_string

    listPorts = serial.tools.list_ports.comports()

    for port in listPorts:
        if port.vid == 6790 and port.pid == 29987 and get_device_id(port.device) == id_string:
            return port.device
    
    raise Exception("Unable to find an interface module with the ID " + id_string)
    



def main():
    
    if len(sys.argv) < 2:
        print "Unable to start client with no identification string."
        print "Usage: client.py <identification string>"
        sys.exit(1)
    

    serialport = find_serial_port(sys.argv[1])

    ser = serial.Serial(serialport, 57600, timeout=0.5)
    time.sleep(2)

    while True:
        ser.write(json.dumps({'cmd':'data'}))
        dataJson = ser.read(65536)
        if dataJson:
            message = json.loads(dataJson)
            print(message)
        time.sleep(polling_frequency)
    

if __name__ == "__main__": main()



    
    