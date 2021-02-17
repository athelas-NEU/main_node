#!/usr/bin/env python3

import sys
import rospy
from main_node.srv import GetKeypoint

def get_keypoint_client(location):
    rospy.wait_for_service('get_keypoint')
    try:
        get_keypoint = rospy.ServiceProxy('get_keypoint', GetKeypoint)
        resp1 = get_keypoint(location)
        return resp1.x, resp1.y
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [location]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        location = sys.argv[1]
    else:
        print(usage())
        sys.exit(1)
    print(f"Requesting {location}")
    x, y = get_keypoint_client(location)
    print(f"Location: [{x}, {y}]")