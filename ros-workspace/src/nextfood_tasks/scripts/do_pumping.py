#! /usr/bin/env python

import roslib
from datetime import datetime, timedelta
roslib.load_manifest('nextfood_tasks')
import rospy
import actionlib
import time
from client_pwm_driver_x6.srv import OperatePwm
from client_relay_x1.srv import OperateRelay

from nextfood_tasks.msg import *


class DoWaterPumpingServer:


  _pump_startup_time = 0
  _device_valve_1 = 1
  _device_valve_2 = 2
  _device_valve_3 = 3

  def __init__(self, name):
    self._action_name = name
    self.connect_server()

  def connect_server(self):
    rospy.loginfo("Registering and starting action server: " + self._action_name)
    self._server = actionlib.SimpleActionServer(self._action_name, DoWaterPumpingAction, self.execute, False)
    self._server.start()

  def execute(self, goal):
    success = True
    rospy.loginfo("Starting Water Pumping. Water Pumping for {} s".format(goal.water_pumping_time))

    set_pump = None
    try:
        set_pump = rospy.ServiceProxy('growbox_water_pump/relay', OperateRelay)
        pump_time = time.time()
        # Setup
        rospy.loginfo("Enabling Water Pumping.")
        set_pump(True)
        time.sleep(goal.water_pumping_time)
        set_pump(False)
        rospy.loginfo("Shutting off Water Pump: Elapsed {} s".format(time.time()-pump_time))
        total_pump_time = time.time() - pump_time

        res = DoWaterPumpingResult()
        res.total_pump_time = total_pump_time
        rospy.loginfo('Water Pumping action server has finished its goal. Total water pump time {} s.'.format(total_pump_time))
        self._server.set_succeeded(res)

    except rospy.ServiceException, e:
        rospy.logerr("Service call for Water Pumping failed: {}".format(e))
        if set_pump:
            set_pump(False)
        self.connect_server()
    except BaseException as e:
        rospy.logerr("Water Pumping execution failed: " + str(e))
        if set_pump:
            set_pump(False)
        self.connect_server()



if __name__ == '__main__':
  rospy.init_node('water_pumping_server')
  server = DoWaterPumpingServer(rospy.get_name())
  rospy.spin()
