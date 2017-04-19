#!/usr/bin/env python
import time
import sys
import rospy
from subprocess import call
from std_msgs.msg import String

class Photographer:

  def __init__(self, name, cam_device):
    self._name = name
    self._cam_device = cam_device

  def run(self):
      pub = rospy.Publisher('growbox_photo', String, queue_size=10)
      while not rospy.is_shutdown():
          rospy.loginfo("Taking photograph.")
          now = time.strftime("%Y%m%d%H%M%S")
          filepath = self._name + "_" + now + ".jpg"
          call(["fswebcam", "-d", self._cam_device, "-r", "1280x960", "--jpeg", "85", "-F", "5", filepath])

          pub.publish(filepath)
          rospy.sleep(5)

def main():
    rospy.init_node('task_photographer', anonymous=True)

    if len(sys.argv) < 3:
          rospy.loginfo("Unable to start photographer. Missing arguments.")
          rospy.loginfo("Usage: do_photograph.py <identification name> <cam device>")
          sys.exit(1)



    try:
        ph = Photographer(sys.argv[1], sys.argv[2])
        ph.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
