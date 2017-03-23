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


class DoWateringServer:


  _pump_startup_time = 1
  _device_valve_1 = 0
  _device_valve_2 = 1
  _device_valve_3 = 2

  def __init__(self, name):
    self._action_name = name
    self._server = actionlib.SimpleActionServer(self._action_name, DoWateringAction, self.execute, False)
    self._server.start()
  
  def execute(self, goal):

    success = True

    rospy.loginfo("Starting Watering. Watering for {} s".format(goal.branch_watering_time))

    set_valve = rospy.ServiceProxy('growbox_water_valves/pwm', OperatePwm)
    set_pump = rospy.ServiceProxy('growbox_water_pump/relay', OperateRelay)

    try:
        # Setup
        set_pump(False)
        set_valve(self._device_valve_1,100)
        set_valve(self._device_valve_2,0)
        set_valve(self._device_valve_3,0)
        # 1st Branch
        set_pump(True)
        time.sleep(self._pump_startup_time)
        timer = time.time()
        pump_time = time.time()
        sleep_dur = goal.branch_watering_time+timer-time.time()
        if sleep_dur > 0: 
            time.sleep(sleep_dur)
        # 2nd Branch
        timer = time.time()
        set_valve(self._device_valve_2,100)
        set_valve(self._device_valve_1,0)
        sleep_dur = goal.branch_watering_time+timer-time.time()
        if sleep_dur > 0: 
            time.sleep(sleep_dur)
        # 3nd Branch
        timer = time.time()
        set_valve(self._device_valve_3,100)
        set_valve(self._device_valve_2,0)
        sleep_dur = goal.branch_watering_time+timer-time.time()
        if sleep_dur > 0: 
            time.sleep(sleep_dur)


        # Finalize
        set_pump(False)
        total_pump_time = time.time() - pump_time
        set_valve(self._device_valve_1,0)
        set_valve(self._device_valve_2,0)
        set_valve(self._device_valve_3,0)

        res = DoWateringResult()
        res.total_pump_time = total_pump_time
        rospy.loginfo('Watering action server has finished its goal. Total water pump time {} s.'.format(total_pump_time))
        self._server.set_succeeded(res)

    except rospy.ServiceException, e:
        rospy.logerror("Service call for watering failed: {}".format(e))
        set_pump(False)
        set_valve(self._device_valve_1,0)
        set_valve(self._device_valve_2,0)
        set_valve(self._device_valve_3,0)




if __name__ == '__main__':
  rospy.init_node('watering_server')
  server = DoWateringServer(rospy.get_name())
  rospy.spin()
