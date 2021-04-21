#!/usr/bin/env python3

from main_node.srv import GetKeypoint
from std_msgs.msg import Int32, Float32, Bool, Int16, Float32MultiArray, String
import time

import queue
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

class SafetyException(Exception):
	pass

class State(object):
	"""
	State parent class.
	"""
	BIOSENSOR_MAP = {
		"pulse": {"keypoint": "hand", "topic": "heart", "distance": 10, "sample_rate":20.0, "sample_interval":125},
		"o2": {"keypoint": "hand", "topic": "spo2", "distance": 10, "sample_rate":20.0, "sample_interval":125},
		"temp": {"keypoint": "forehead", "topic": "temp", "distance": 6, "sample_rate":20.0, "sample_interval":125},
		"stethoscope": {"keypoint": "chest", "topic": "/biosensors/stethoscope", "distance": 0,"sample_rate": 44100.0, "sample_interval":200},
	}

	AXES_SETTINGS = {
		"pulse": {"ylim": [40, 200], "color": (0.9, 0.5, 0.2), "ylabel": "Average Pulse"},
		"o2": {"ylim": [90, 101], "color": (0, 0.1, 0.8), "ylabel": "O2 %"},
		"temp": {"ylim": [70, 105], "color":(0.7, 0, 0), "ylabel": "Temperature"},
		"stethoscope": {"ylim": [-0.4, 0.4], "color":(0, 1, 0.29), "ylabel": "Amplitude"}
	}

	def __init__(self):
		self.stop = False
		print(f"Current state: {str(self)}")
		pub_state = rospy.Publisher('current_state', String, queue_size=10)
		pub_state.publish(str(self))

	def execute(self):
		if self.stop:
			self.stop = False
			raise SafetyException
		pass

	def __repr__(self):
		return self.__str__()

	def __str__(self):
		"""
		Returns the name of the State.
		"""
		return self.__class__.__name__
	
	def start_safety_monitoring(self):
		self.sub_distance = rospy.Subscriber("distance", Float32, self.__safety_callback)

	def stop_safety_monitoring(self):
		self.sub_distance.unregister()

	def __safety_callback(self, data):
		if data.data > 470 or data.data < 2:
			pub_stop = rospy.Publisher('arm_control/stop', Bool, queue_size=10)
			pub_stop.publish(True)
			self.stop = True 


class Idle(State):
	"""
	Waits for command, then transitions to PostionArm state.
	"""

	def __init__(self):
		super().__init__()
		pub_reset = rospy.Publisher('arm_control/reset', Bool, queue_size=10)
		pub_reset.publish(True)
		pub_reset.publish(True)

		self.sub_test = rospy.Subscriber("start_test", String, self.__input_callback)
		self.biosensor = None
		print("waiting for biosensor test input")
	
	def execute(self):
		super().execute()
		if self.biosensor:
			return PositionArmXY(self.biosensor)
		return self
	
	def __input_callback(self, data):
		self.biosensor = data.data
		print(self.biosensor)

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
		# self.start_safety_monitoring()

	def execute(self):
		super().execute()
		get_keypoint = rospy.ServiceProxy('get_keypoint', GetKeypoint)
		resp = get_keypoint(self.location)
		print(f"Location of {self.location} is {resp.x},{resp.y}")
		# If the keypoint is not found, move backwards
		if resp.x == -999:
			self.pub_z.publish(-10)
			return self

		print("x")
		print(-1 * resp.x / 5)
		print("y")
		print(-1 * resp.y / 5)

		# If centered, transition states
		if abs(resp.x) < 28 and abs(resp.y) < 28:
			print("centered")
			if self.location == "forehead":
				self.pub_y.publish(50)
			return PositionArmZ(self.sensor)

		# Only move x if x is not centered
		if abs(resp.x) >= 28 and abs(resp.x) < 100:
			if resp.x < 0:
				self.pub_x.publish(-5)
			elif resp.x > 0:
				self.pub_x.publish(5)
			time.sleep(0.5)
		
		# Move y
		self.pub_y.publish(-1 * int(resp.y/5))
		
		time.sleep(3.5)
		return self

class PositionArmZ(State):
	"""
	Position arm in the z direction.
	"""

	def __init__(self, sensor):
		super().__init__()
		# self.stop_safety_monitoring()
		self.sensor = sensor
		self.distance = self.BIOSENSOR_MAP[self.sensor]["distance"]
		self.pub_z = rospy.Publisher('arm_control/z', Int16, queue_size=10)
		self.pub_y = rospy.Publisher('arm_control/y', Int16, queue_size=10)
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
			if self.sensor == "pulse" or self.sensor == "o2":
				print("going down")
				self.pub_y.publish(-100)
				time.sleep(3)
			return BioData(self.sensor)
		else:
			# TODO: return self
			return self
	
	def __distance_callback(self, data):
		if time.time() - self.time > 3 and self.positioned is False:
			self.time = time.time()
			print(f"Distance: {data.data} in")
			if data.data <= self.distance:
				self.positioned = True
			elif data.data > 470:
				print("close to chest")
				self.positioned = True
			else:
				print("going forward: distance")
				self.pub_z.publish(40)
				self.positioned = False	
	
	def __pressure_callback(self, data):
		if self.sensor == "stethoscope" and self.positioned is True:
			if time.time() - self.time > 3:
				self.time = time.time()
				print(f"Pressure: {data.data}")
				self.positioned_pressure = data.data > 10
				if not self.positioned_pressure:
					print("going forward: pressure")
					self.pub_z.publish(10)


class BioData(State):
	"""
	Collect bio data for 15 seconds.
	"""

	def __init__(self, sensor):
		super().__init__()
		self.sensor = sensor

		print("in init")
		self.start_time = time.time()

	def execute(self):
		super().execute()
		if time.time() - self.start_time > 15:
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

class Stop(State):

	def __init__(self):
		super().__init__()
		self.reset = False
		self.sub_reset = rospy.Subscriber("arm_control/reset", Bool, self.__reset_callback)

	def execute(self):
		super().execute()
		if self.reset:
			return Idle()
		return self
	
	def __reset_callback(self, data):
		if data.data == True:
			self.reset = True