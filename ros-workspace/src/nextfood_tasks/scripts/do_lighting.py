#! /usr/bin/env python

import roslib
from datetime import datetime, timedelta
roslib.load_manifest('nextfood_tasks')
import rospy
import time
import actionlib
from client_pwm_driver_x6.srv import *

from nextfood_tasks.msg import *

def secPrettyPrint(seconds):
    m, s = divmod(seconds, 60)
    h, m = divmod(m, 60)
    return "Hour(s): {} Min(s): {} Sec(s): {}".format(h,m,s)

class DoLightingServer:
  def __init__(self, name):
    self._action_name = name
    self._server = actionlib.SimpleActionServer(self._action_name, DoLightingAction, self.execute, False)
    self._server.start()
  
  def set_lights(self, white, red, blue):
    white_device = 1
    red_device = 2
    blue_device = 3
    rospy.wait_for_service('growbox_led_driver/pwm')
    try:
        set_light = rospy.ServiceProxy('growbox_led_driver/pwm', OperatePwm)
        set_light(white_device,white)
        set_light(red_device,red)
        set_light(blue_device,blue)
    except rospy.ServiceException, e:
        rospy.logerr("Service call to 'growbox_led_driver/pwm' failed: {}".format(e))

  def execute(self, goal):

    success = True
    r = rospy.Rate(1) # Resolution of lighting in 1 sec

    white = goal.white_intensity
    red = goal.red_intensity
    blue = goal.blue_intensity

    rospy.loginfo("Starting lighting W: {}% R: {}% B: {}%.".format(white,red,blue))
    rospy.loginfo("Running lighting for {}".format(secPrettyPrint(goal.runtime)))

    start_time = time.time()

    for i in range(1, int(goal.runtime)):
        # check that preempt has not been requested by the client
        if self._server.is_preempt_requested():
            rospy.loginfo('Service {} has been preempted. Stopping the lighting.'.format(self._action_name))
            self._server.set_preempted()
            success = False
            break

        self.set_lights(white,red,blue)

        fb = DoLightingFeedback()
        fb.runtime_completed = i
        fb.current_white_intensity = white
        fb.current_red_intensity = red
        fb.current_blue_intensity = blue
        self._server.publish_feedback(fb)
        r.sleep()

    self.set_lights(0,0,0)

    if success:
            res = DoLightingResult()
            res.total_runtime = time.time() - start_time
            rospy.loginfo('The light action server has finished its goal. Lights ran for {}'.format(secPrettyPrint(res.total_runtime)))
            self._server.set_succeeded(res)


if __name__ == '__main__':
  rospy.init_node('lighting_server')
  server = DoLightingServer(rospy.get_name())
  rospy.spin()
