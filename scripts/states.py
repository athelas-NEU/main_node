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
		"temp": {"keypoint": "forehead", "topic": "temp", "distance": 1},
		"stethoscope": {"keypoint": "chest", "topic": "stethoscope", "distance": 0},
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
		pub_reset = rospy.Publisher('arm_control/reset', Bool, queue_size=10)
		pub_reset.publish(True)
		pub_reset.publish(True)

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
		self.pub_z = rospy.Publisher('arm_control/z', Int16, queue_size=10)
		pub_reset = rospy.Publisher('arm_control/reset', Bool, queue_size=10)
		pub_reset.publish(True)
		pub_reset.publish(True)
		time.sleep(2)

	def execute(self):
		super().execute()
		get_keypoint = rospy.ServiceProxy('get_keypoint', GetKeypoint)
		resp = get_keypoint(self.location)
		# TODO: convert image coordinates to centimeters
		if resp.x == -999:
			self.pub_z.publish(-1)
			return self
		print(-1 * resp.x)
		print(-1 * resp.y)
		if abs(resp.x) < 28 and abs(resp.y) < 28:
			print("centered")
			return PositionArmZ(self.sensor)
		# Range from -4 to 4
		self.pub_x.publish(-1 * int(resp.x / 28))
		self.pub_y.publish(-1 * int(resp.y / 28))
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
		self.pub_z = rospy.Publisher('arm_control/z', Int16, queue_size=10)
		self.positioned = False
		self.positioned_pressure = False if self.sensor == "stethoscope" else True
		self.sub_distance = rospy.Subscriber("distance", Float32, self.__distance_callback)
		self.sub_pressure = rospy.Subscriber("pressure", Int16, self.__pressure_callback)
		self.time = time.time()

	def execute(self):
		super().execute()
		if self.positioned and self.positioned_pressure:
			self.sub_distance.unregister()
			self.sub_pressure.unregister()
			return BioData(self.sensor)
		else:
			# TODO: return self
			return self
	
	def __distance_callback(self, data):
		if time.time() - self.time > 1 and self.positioned is False:
			self.time = time.time()
			print(f"Distance: {data.data} in")
			if abs(data.data - self.distance) < 1.5:
				self.positioned = True
			elif data.data > 470:
				print("close to chest")
				self.positioned = True
			else:
				self.pub_z.publish(1)
				self.positioned = False
	
	def __pressure_callback(self, data):
		if self.sensor == "stethoscope" and self.positioned is True:
			if time.time() - self.time > 1:
				self.time = time.time()
				print(f"Pressure: {data.data}")
				self.positioned_pressure = data.data > 12
				if not self.positioned_pressure:
					self.pub_z.publish(1)

class BioData(State):
	"""
	Collect bio data for 15 seconds.
	"""

	def __init__(self, sensor):
		super().__init__()
		self.sensor = sensor
		topic = self.BIOSENSOR_MAP[self.sensor]["topic"]
		self.start_time = time.time()
		self.sub = rospy.Subscriber(f"{topic}", Float32, self.__bio_callback)

	def execute(self):
		super().execute()
		if time.time() - self.start_time > 15:
			self.sub.unregister()
			# Tell arm to reset
			print("go back")
			pub_z = rospy.Publisher('arm_control/z', Int16, queue_size=10)
			pub_z.publish(-5)
			time.sleep(0.5)
			print("reseting arm")
			pub_reset = rospy.Publisher('arm_control/reset', Bool, queue_size=10)
			pub_reset.publish(True)
			time.sleep(0.5)
			return Idle()
		else:
			return self
	
	def __bio_callback(self, data):
		print(data.data + 12)
		