#!/usr/bin/env python3

from main_node.srv import GetKeypoint
from std_msgs.msg import Int32, Float32, Bool, Int16
from main_node.msg import biodata_array
import time

import queue
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

import sounddevice as sd



class State(object):
	"""
	State parent class.
	"""
	BIOSENSOR_MAP = {
		"pulse": {"keypoint": "hand", "topic": "pulseox/heart", "distance": 10, "samplerate":100.0},
		"o2": {"keypoint": "hand", "topic": "pulseox/o2", "distance": 10, "samplerate":100.0},
		"temp": {"keypoint": "forehead", "topic": "temp", "distance": 10, "samplerate":100.0},
		"stethoscope": {"keypoint": "chest", "topic": "stethoscope", "distance": 10,"samplerate": 44100.0},
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

		# biosensor = input("Enter the biosensor name: ")
		biosensor = "stethoscope"
		print(biosensor)
		print("DEBUG: going to biosensor state")
		return BioData(biosensor)

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
		print(resp.x)
		print(-1 * resp.y)
		if abs(resp.x) < 28 and abs(resp.y) < 28:
			print("centered")
			return PositionArmZ(self.sensor)
		# Range from -4 to 4
		self.pub_x.publish(int(resp.x / 28))
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



def plot_callback(frame, _state):
	"""
	Fields needed from BioData:
		_state.data_q
		_state.plotdata
		_state.lines
	"""
	while True:
		try:
			data = _state.data_q.get_nowait()
		except queue.Empty:
			break
		shift = len(data)
		plotdata = np.roll(_state.plotdata, -shift, axis=0)
		 # Elements that roll beyond the last position are re-introduced

		# npdata = np.asarray(data)[0:shift]
		print("plot shape: ", data.shape)

		plotdata[-shift:,:] = data
	for column, line in enumerate(_state.lines):
		line.set_ydata(_state.plotdata[:,column])
	return _state.lines

class BioData(State):
	"""
	Collect bio data for 15 seconds.
	"""

	def __init__(self, sensor):
		super().__init__()
		self.sensor = sensor
		topic = self.BIOSENSOR_MAP[self.sensor]["topic"]

		window = 1000
		downsample = 1
		interval = 30
		samplerate = self.BIOSENSOR_MAP[self.sensor]["samplerate"]
		length = int(window * samplerate / 1000 * downsample)

		self.data_q = queue.Queue()

		print("Sample Rate found: ", samplerate)

		self.plotdata = np.zeros((length, 1))
		print("plotdata shape found: ", self.plotdata.shape)
		self.fig, self.ax = plt.subplots(figsize=(8,4))

		self.lines = self.ax.plot(self.plotdata,color = (0,1,0.29))

		self.ax.set_title("Athelas: " + str(self.sensor) + " data")
		self.ax.set_facecolor((0,0,0))
		self.ax.set_yticks([0])
		self.ax.yaxis.grid(True)

		### This should be changed to be modular for diff devices
		self.ani = FuncAnimation(self.fig, plot_callback, fargs=(self,), interval=interval, blit=True)
		plt.show()

		self.start_time = time.time()
		self.sub = rospy.Subscriber(f"/biosensors/{topic}", Float32, self.__bio_callback)
		print("finished state init")

	def execute(self):
		plt.show()

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
		self.data_q.put(data)