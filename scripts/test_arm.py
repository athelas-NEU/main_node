#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int16, Bool

from states import Idle


def main_loop():
  # Create ROS node
  rospy.init_node('test_arm', anonymous=True)
  pub_z = rospy.Publisher('arm_control/x', Int16, queue_size=10)
  pub_reset = rospy.Publisher('arm_control/reset', Bool, queue_size=10)
  for i in range(3):
    pub_z.publish(-2)
    time.sleep(1)
  pub_reset.publish(True)
  


if __name__ == "__main__":
  main_loop()