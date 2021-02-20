import rospy

import states.state import State
from main_node.srv import GetKeypoint
from std_msgs.msg import Int32
import scripts.main_node


class PostionArmXY(State):
  """
  
  """
  BIOSENSOR_TO_KEYPOINT = {
    "pulse": "hand",
    "o2": "hand",
    "temp": "forehead",
    "stethoscope": "chest"
  }

  def __init__(self, sensor):
    super().__init__()
    rospy.wait_for_service('get_keypoint')
    self.location = BIOSENSOR_MAP[sensor]
    self.pub_x = rospy.Publisher('/arm_control/x', Int32, queue_size=10)
    self.pub_y = rospy.Publisher('/arm_control/y', Int32, queue_size=10)

  def execute(self):
    super().execute()
    get_keypoint = rospy.ServiceProxy('get_keypoint', GetKeypoint)
    resp1 = get_keypoint(self.location)
    self.pub_x.publish(resp1.x)
    self.pub_y.publish(resp1.y)
    print(resp1.x)
    print(resp1.y)
    main_node.state = Idle()


    