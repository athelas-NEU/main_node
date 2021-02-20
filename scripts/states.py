from main_node.srv import GetKeypoint
from std_msgs.msg import Int32, Float32, Bool, Int16
import rospy
import time


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
		self.pub_x = rospy.Publisher('arm_control/x', Int16, queue_size=10)
		self.pub_y = rospy.Publisher('arm_control/y', Int16, queue_size=10)

	def execute(self):
		super().execute()
		get_keypoint = rospy.ServiceProxy('get_keypoint', GetKeypoint)
		resp = get_keypoint(self.location)
		# TODO: convert image coordinates to centimeters
		if resp.x == -999:
			pub_reset = rospy.Publisher('arm_control/reset', Bool, queue_size=10)
			pub_reset.publish(True)
			time.sleep(2)
			return self
		self.pub_x.publish(int(resp.x / 20))
		self.pub_y.publish(-1 * int(resp.y / 20))
		print(resp.x)
		print(-1 * resp.y)
		if abs(resp.x) < 15 and abs(resp.y) < 15:
			print("centered")
			return PositionArmZ(self.sensor)
		# TODO: only move to next state once centered
		time.sleep(1)
		return self

class PositionArmZ(State):
	"""
	Position arm in the z direction.
	"""

	def __init__(self, sensor):
		super().__init__()
		self.sensor = sensor
		self.distance = self.BIOSENSOR_MAP[self.sensor]["distance"]
		self.pub_z = rospy.Publisher('/arm_control/z', Int16, queue_size=10)
		self.positioned = False
		self.sub = rospy.Subscriber("/distance_sensor", Float32, self.__distance_callback)

	def execute(self):
		super().execute()
		if self.positioned:
			self.sub.unregister()
			return BioData(self.sensor)
		else:
			# TODO: return self
			return BioData(self.sensor)
	
	def __distance_callback(self, data):
		print(data.distance)
		# TODO: decide if positioned
		self.pub_z.publish(data - self.distance)
		self.positioned = True

class BioData(State):
	"""
	Collect bio data for 15 seconds.
	"""

	def __init__(self, sensor):
		super().__init__()
		self.sensor = sensor
		rospy.wait_for_service('get_keypoint')
		topic = self.BIOSENSOR_MAP[self.sensor]["topic"]
		self.start_time = time.time()
		self.sub = rospy.Subscriber(f"/biosensors/{topic}", Float32, self.__bio_callback)

	def execute(self):
		super().execute()
		if time.time() - self.start_time > 15:
			self.sub.unregister()
			# Tell arm to reset
			print("reseting arm")
			pub_reset = rospy.Publisher('arm_control/reset', Bool, queue_size=10)
			pub_reset.publish(True)
			time.sleep(0.5)
			return Idle()
		else:
			return self
	
	def __bio_callback(self, data):
		print(data)
		