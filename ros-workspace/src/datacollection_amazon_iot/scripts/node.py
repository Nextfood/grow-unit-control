#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import json
from subprocess import call
import time
import rospkg
import datetime
import signal
import sys

mosqparam = [
 '--cafile', '/resources/amazon-aws-iot-connection/certs/root-CA.crt',
 '--cert', '/resources/amazon-aws-iot-connection/certs/cert.pem',
 '--key', '/resources/amazon-aws-iot-connection/certs/private.key',
 '-h', open("/resources/amazon-aws-iot-connection/amazon-url").readline().rstrip(),
 '-p', '8883'
]

gotData = False
databaseData = {
    'light_data': {
          'growbox' : {}
    },
    'env_data': {
          'growbox' : {}
    }
}


def signal_handler(signal, frame):
        print('Exiting program')
        sys.exit(0)


def callback_light_data_2591_full(data):
     global databaseData, gotData
     databaseData['light_data']['growbox']['full'] = data.data
     gotData = True

def callback_light_data_2591_ir(data):
     global databaseData, gotData
     databaseData['light_data']['growbox']['ir'] = data.data
     gotData = True

def callback_light_data_2591_lux(data):
     global databaseData, gotData
     databaseData['light_data']['growbox']['lux'] = data.data
     gotData = True
    
def listener():


    rospy.init_node('datacollection_amazon_iot', anonymous=True)

    rospy.Subscriber("/light_data/TSL2591/full", Float64, callback_light_data_2591_full)
    rospy.Subscriber("/light_data/TSL2591/ir", Float64, callback_light_data_2591_ir)
    rospy.Subscriber("/light_data/TSL2591/lux", Float64, callback_light_data_2591_lux)


    while True:
        if gotData:
            databaseData['datetime'] = datetime.datetime.utcnow().isoformat()
            databaseData['env_data']['growbox']['temperature'] = 23.1
            databaseData['env_data']['growbox']['humidity'] = 44.4
            databaseData['env_data']['growbox']['pressure'] = 1012.4
            #print "Post data: " + json.dumps(databaseData)
            call(['mosquitto_pub'] + mosqparam + ['-t', 'grow-box-1/sensors', '-m', json.dumps(databaseData)])
        time.sleep(10)

    rospy.spin()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    listener()

