from main_node.srv import GetKeypoint
from std_msgs.msg import Int32, Float32
import rospy


class State(object):
	"""
	State parent class.
	"""
	BIOSENSOR_MAP = {
		"pulse": {"keypoint": "hand", "topic": "pulseox/heart", "distance": 10},
		"o2": {"keypoint": "hand", "topic": "pulseox/o2", "distance": 10},
		"temp": {"keypoint": "forehead", "topic": "temp", "distance": 10},
		"stethoscope": {"keypoint": "chest", "topic": "stethoscope", "distance": 10},
	}

	def __init__(self):
		print(f"Current state: {str(self)}")

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
	Position arm in the x and y direction.
	"""

	def __init__(self, sensor):
		super().__init__()
		self.sensor = sensor
		rospy.wait_for_service('get_keypoint')
		self.location = self.BIOSENSOR_MAP[self.sensor]["keypoint"]
		self.pub_x = rospy.Publisher('/arm_control/x', Int32, queue_size=10)
		self.pub_y = rospy.Publisher('/arm_control/y', Int32, queue_size=10)

	def execute(self):
		super().execute()
		get_keypoint = rospy.ServiceProxy('get_keypoint', GetKeypoint)
		resp1 = get_keypoint(self.location)
		# TODO: convert image coordinates to centimeters
		self.pub_x.publish(resp1.x)
		self.pub_y.publish(resp1.y)
		print(resp1.x)
		print(resp1.y)
		# TODO: only move to next state once centered
		return PositionArmZ(self.sensor)

class PositionArmZ(State):
	"""
	Position arm in the z direction.
	"""

	def __init__(self, sensor):
		super().__init__()
		self.sensor = sensor
		rospy.wait_for_service('get_keypoint')
		self.distance = self.BIOSENSOR_MAP[self.sensor]["distance"]
		self.pub_z = rospy.Publisher('/arm_control/z', Int32, queue_size=10)
		self.positioned = False
		rospy.Subscriber("/distance_sensor", Float32, self.__distance_callback)

	def execute(self):
		super().execute()
		if self.positioned:
			return Idle()
		else:
			# TODO: return self
			return Idle()
	
	def __distance_callback(self, data):
		print(data.distance)
		# TODO: decide if positioned
		self.pub_z.publish(data.distance - self.distance)
		self.positioned = True
