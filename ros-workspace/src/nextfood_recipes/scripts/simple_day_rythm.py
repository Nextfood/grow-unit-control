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
    'lighting_on_start_time': "19:05:00",  # [immediately] or [hh:mm:ss]
#    'lighting_on_start_time': "immediately",  #[immediately] or [hh:mm:ss]
    'lighting_on_frequency': 10,  # [s]
    # 'lighting_on_frequency': 24 * 3600, # [s]
    'lighting_on_duration': 5,  # [s]
    # 'lighting_on_duration': 16 * 3600, # [s]

    'watering_on_start_time': "00:00:00",  # [immediately] or [hh:mm:ss]
    'watering_on_frequency': 5 * 60,  # [s]
    'watering_on_duration': 0.5,  # [s]
}


class ActionClient:

    def __init__(self, name, start, frequency, duration, action_method):
        self._name = name
        self._start_str = start
        self._frequency = frequency
        self._duration = duration
        self._action_method = action_method
        client = actionlib.SimpleActionClient('lighting_server', DoLightingAction)
        client.wait_for_server()

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
	            start_time = rospy.Time()
        	    self._action_method(self._duration)
                    rospy.sleep(rospy.Time() - start_time + rospy.Duration.from_sec(self._frequency))

        except rospy.ROSInterruptException:
            rospy.logwarn("The light client recipe was interrupted.")

class Actions:

	def __init__(self):
	    self._light_client = actionlib.SimpleActionClient('lighting_server', DoLightingAction)
            self._light_client.wait_for_server()
            self._water_client = actionlib.SimpleActionClient('watering_server', DoWateringAction)
            self._water_client.wait_for_server()

	def lighting_on(self, runtime):
	    # Creates a goal to send to the action server.
	    goal = DoLightingGoal()
	    goal.runtime = runtime
	    goal.white_intensity = 100
	    goal.red_intensity = 100
	    goal.blue_intensity = 100
	    self._light_client.send_goal(goal)
	    self._light_client.wait_for_result()

	    return self._light_client.get_result()


	def watering_on(self, runtime):
	    # Creates a goal to send to the action server.
	    goal = DoWateringGoal()
	    goal.branch_watering_time = runtime
	    self._water_client.send_goal(goal)
	    self._water_client.wait_for_result()
	    return self._water_client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('simple_day_rythm')

        rospy.loginfo("Starting simple day rythm.")

        actions = Actions()

        lighting_client = ActionClient("Lighting", setup['lighting_on_start_time'],
                                       setup['lighting_on_frequency'], setup['lighting_on_duration'], actions.lighting_on)
        lighting_thread = threading.Thread(target=lighting_client.run)
        lighting_thread.daemon = True
        lighting_thread.start()


#        watering_client = ActionClient("Watering", setup['watering_on_start_time'],
#		setup['watering_on_frequency'], setup['watering_on_duration'], watering_on)
#	    watering_thread = threading.Thread(target=watering_client.run)
#	    watering_thread.daemon = True
#	    watering_thread.start()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logwarn("The simple_day_rythm script was interrupted.")
