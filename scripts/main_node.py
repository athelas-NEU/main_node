#!/usr/bin/env python3

import rospy

from scripts.states.idle import Idle


def main_loop():
  # Create ROS node
  rospy.init_node('main_node', anonymous=True)
  global state
  state = Idle()
  while(1) {
    state.execute()
    sleep(0.5)
  }


if __name__ == "__main__":
  main_loop()