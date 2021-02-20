#!/usr/bin/env python3

import rospy
import time

from states import Idle


def main_loop():
  # Create ROS node
  rospy.init_node('main_node', anonymous=True)
  state = Idle()
  while(1):
    state = state.execute()
    time.sleep(0.5)


if __name__ == "__main__":
  main_loop()