#! /usr/bin/env python

import roslib
from datetime import datetime, timedelta
roslib.load_manifest('nextfood_tasks')
import rospy
import actionlib
import time
from client_pwm_driver_x6.srv import *
from client_relay_x1.srv import OperateRelay

from nextfood_tasks.msg import *


class DoWateringServer:

  def __init__(self, name):
    self._action_name = name
    self._server = actionlib.SimpleActionServer(self._action_name, DoWateringAction, self.execute, False)
    self._server.start()

  def run_branch(self, set_valves, log_text, valves, watering_time):
      timer = time.time()
      set_valves(valves)
      rospy.loginfo(log_text)
      sleep_dur = watering_time * 2 + timer-time.time() # The 3rd branch is actually 2 branches. Give double time.
      if sleep_dur > 0:
          time.sleep(sleep_dur)

  def execute(self, goal):

    success = True

    rospy.loginfo("Starting Watering. Watering for {} s".format(goal.branch_watering_time))

    set_valves = rospy.ServiceProxy('growbox_water_valves/pwms', OperatePwms)
    set_pump = rospy.ServiceProxy('growbox_water_pump/relay', OperateRelay)

    try:
        # Setup
        set_pump(False)
        set_valves([100,0,0,0,0,0])
        set_pump(True)
        total_pump_time = time.time()
        self.run_branch(set_valves, "Enabling 1st watering branch. Elapsed: {} s".format(time.time()-total_pump_time), [100,0,0,0,0,0], goal.branch_watering_time)
        self.run_branch(set_valves, "Enabling 2nd watering branch. Elapsed: {} s".format(time.time()-total_pump_time), [0,100,0,0,0,0], goal.branch_watering_time)
        self.run_branch(set_valves, "Enabling 3rd watering branch. Elapsed: {} s".format(time.time()-total_pump_time), [0,0,100,0,0,0], goal.branch_watering_time)
        self.run_branch(set_valves, "Enabling 4th watering branch. Elapsed: {} s".format(time.time()-total_pump_time), [0,0,0,100,0,0], goal.branch_watering_time)
        self.run_branch(set_valves, "Enabling 5th watering branch. Elapsed: {} s".format(time.time()-total_pump_time), [0,0,0,0,100,0], goal.branch_watering_time)
        self.run_branch(set_valves, "Enabling 6th watering branch. Elapsed: {} s".format(time.time()-total_pump_time), [0,0,0,0,0,100], goal.branch_watering_time)

        # Finalize
        set_pump(False)
        rospy.loginfo("Shutting off water: Elapsed {} s".format(time.time()-total_pump_time))
        set_valves([0,0,0,0,0,0])

        res = DoWateringResult()
        res.total_pump_time = time.time() - total_pump_time
        rospy.loginfo('Watering action server has finished its goal. Total water pump time {} s.'.format(time.time() - total_pump_time))
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
