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


  _pump_startup_time = 0
  _device_valve_1 = 1
  _device_valve_2 = 2
  _device_valve_3 = 3

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
        rospy.loginfo("Enabling 1st watering branch.")
        set_valve(self._device_valve_2,0)
        set_valve(self._device_valve_3,0)
        set_valve(self._device_valve_1,100)
        time.sleep(0.5)


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
        rospy.loginfo("Enabling 2st watering branch: Elapsed {} s".format(time.time()-pump_time))
        set_valve(self._device_valve_2,100)
        set_valve(self._device_valve_1,0)
        sleep_dur = goal.branch_watering_time+timer-time.time()
        if sleep_dur > 0: 
            time.sleep(sleep_dur)

        # 3rd Branch
        timer = time.time()
        rospy.loginfo("Enabling 3rd watering branch: Elapsed {} s".format(time.time()-pump_time))
        set_valve(self._device_valve_3,100)
        set_valve(self._device_valve_2,0)
        sleep_dur = goal.branch_watering_time * 2 + timer-time.time() # The 3rd branch is actually 2 branches. Give double time.
        if sleep_dur > 0: 
            time.sleep(sleep_dur)

        # Finalize
        rospy.loginfo("Shutting off water: Elapsed {} s".format(time.time()-pump_time))
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
        rospy.logerr("Service call for watering failed: {}".format(e))
        set_pump(False)
        set_valve(self._device_valve_1,0)
        set_valve(self._device_valve_2,0)
        set_valve(self._device_valve_3,0)




if __name__ == '__main__':
  rospy.init_node('watering_server')
  server = DoWateringServer(rospy.get_name())
  rospy.spin()
