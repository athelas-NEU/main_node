#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int32

from states import Idle


def main_loop():
  # Create ROS node
  rospy.init_node('test_arm', anonymous=True)
  pub_z = rospy.Publisher('/arm_control/z', Int32, queue_size=10)
  while 1:
    pub_z.publish(10)
    time.sleep(0.5)
  


if __name__ == "__main__":
  main_loop()