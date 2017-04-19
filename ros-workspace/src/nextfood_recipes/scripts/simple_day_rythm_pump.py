#! /usr/bin/env python
import threading
import rospy
import actionlib
from nextfood_tasks.msg import *
from datetime import datetime

def str_to_sec(time_str):
    h, m, s = time_str.split(':')
    return int(h) * 3600 + int(m) * 60 + int(s)

def pretty_print_sec(seconds):
   m, s = divmod(seconds, 60)
   h, m = divmod(m, 60)
   return "%d:%02d:%02d" % (h, m, s)


setup = {
    'water_pumping_on_start_time': "immediately",  # [immediately] or [hh:mm:ss]
    'water_pumping_on_frequency': 5 * 60,  # [s]
    'water_pumping_on_duration': 4,  # [s]
}


class ActionClient:

    def __init__(self, name, start, frequency, duration, action_method):
        self._name = name
        self._start_str = start
        self._frequency = frequency
        self._duration = duration
        self._action_method = action_method

    def run_action(self,event):
        self._action_method(self._duration)

    def run(self):
        try:
            rospy.loginfo("Action {} started. Start time: {}, Freq: {} s, Duration: {} s".format(
                self._name, self._start_str, self._frequency, self._duration))

            now = datetime.now()

            if self._start_str.lower() != "immediately":
                seconds_since_midnight = (now - datetime.now().replace(
                    hour=0, minute=0, second=0, microsecond=0)).total_seconds()
                start_time = str_to_sec(self._start_str)
                if start_time >= seconds_since_midnight: # the start time is soon
                    rospy.loginfo("Starting {} action in {}.".format(self._name,pretty_print_sec(start_time - seconds_since_midnight)))
                    rospy.sleep(start_time - seconds_since_midnight)
                else:
                    rospy.sleep(3600*24 + start_time - seconds_since_midnight) # the start time is "tomorrow"
                    rospy.loginfo("Starting {} action in {}.".format(self._name,pretty_print_sec(3600*24 + start_time - seconds_since_midnight)))
            else:
                rospy.loginfo("Starting {} action immediately.".format(self._name))

            while 1:
	            start_time = rospy.get_time()
        	    self._action_method(self._duration)
                    sleeptime = self._frequency - (rospy.get_time() - start_time)
                    rospy.loginfo("Sleeping for {} s before next action.".format(sleeptime))
                    rospy.sleep(sleeptime)

        except rospy.ROSInterruptException:
            rospy.logwarn("The action client recipe was interrupted.")

class Actions:

	def watering_on(self, runtime):
            try:
                    pump_client = actionlib.SimpleActionClient('water_pumping_server', DoWaterPumpingAction)
                    if pump_client.wait_for_server(rospy.Duration(60)) == False:
                           raise RuntimeError("Water pumping server does not seems to be online.")
                    # Creates a goal to send to the action server.
                    goal = DoWaterPumpingGoal()
                    goal.water_pumping_time = runtime
                    pump_client.send_goal(goal)
                    pump_client.wait_for_result()
                    return pump_client.get_result()
            except RuntimeError as e:
                    rospy.logwarn("Timeout occurred: " + str(e))
            except BaseException as e:
                    rospy.logwarn("The water pump action failed: " + str(e))
            return 0

if __name__ == '__main__':
    try:
        rospy.init_node('simple_day_rythm_pump')

        rospy.loginfo("Starting simple day rythm water pump.")

        actions = Actions()

        watering_client = ActionClient("Water Pumping", setup['water_pumping_on_start_time'],
					setup['water_pumping_on_frequency'], setup['water_pumping_on_duration'], actions.watering_on)
        watering_thread = threading.Thread(target=watering_client.run)
        watering_thread.daemon = True
        watering_thread.start()

        rospy.spin()


        rospy.loginfo("The simple day rythm water pump is exiting.")

    except rospy.ROSInterruptException:
        rospy.logwarn("The simple_day_rythm_pump script was interrupted.")
