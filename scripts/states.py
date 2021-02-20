from main_node.srv import GetKeypoint
from std_msgs.msg import Int32
import rospy

class State(object):
  """
  State parent class.
  """

  def __init__(self):
    print(f"Processing current state: {str(self)}")

  def execute(self):
    pass

  def __repr__(self):
    return self.__str__()

  def __str__(self):
    """
    Returns the name of the State.
    """
    return self.__class__.__name__

class Idle(State):
  """
  Waits for command, then transitions to PostionArm state.
  """

  def execute(self):
    super().execute()

    biosensor = input("Enter the biosensor name: ")
    print(biosensor)
    return PositionArmXY(biosensor)

class PositionArmXY(State):
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
    self.location = self.BIOSENSOR_TO_KEYPOINT[sensor]
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
    return Idle()