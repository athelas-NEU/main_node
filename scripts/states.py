#!/usr/bin/env python3

from main_node.srv import GetKeypoint
from main_node.msg import biodatat
from std_msgs.msg import Int32, Float32, Bool, Int16
import time

import queue
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

class State(object):
	"""
	State parent class.
	"""
	BIOSENSOR_MAP = {
		"pulse": {"keypoint": "hand", "topic": "pulseox/heart", "distance": 10, "sample_rate":100.0, "sample_interval":30},
		"o2": {"keypoint": "hand", "topic": "pulseox/o2", "distance": 10, "sample_rate":100.0, "sample_interval":30},
		"temp": {"keypoint": "forehead", "topic": "temp", "distance": 10, "sample_rate":100.0, "sample_interval":30},
		"stethoscope": {"keypoint": "chest", "topic": "stethoscope", "distance": 10,"sample_rate": 44100.0, "sample_interval":30},
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


####################################################
### BioData State Plotting helper functions ########
####################################################

def calculate_shift(data):
	if type(data) == float:
		return 1
	else:
		return len(data)

def plot_callback(frame, state):
	"""
	Fields needed from BioData:
		state.data_q
		state.plotdata
		state.lines
	"""

	while True:
		try:
			data_np = np.asarray(state.data_q.get_nowait())
			data = np.expand_dims(data_np, axis=1)
			# print_list(data)
		except queue.Empty:
			break

		shift = calculate_shift(data)
		state.plotdata = np.roll(state.plotdata, -shift, axis=0)
		 # Elements that roll beyond the last position are re-introduced

		# npdata = np.asarray(data)[0:shift]
		state.plotdata[-shift:,:] = data

	for column, line in enumerate(state.lines):
		line.set_ydata(state.plotdata[:,column])

	return state.lines


class BioData(State):
	"""
	Collect bio data for 15 seconds.
	"""

	def __init__(self, sensor):
		super().__init__()
		self.sensor = sensor
		topic = self.BIOSENSOR_MAP[self.sensor]["topic"]

		print("in init")

		### Begin pylot setup

		window = 1000
		self.downsample = 1

		 # this is update interval in miliseconds for plot (less than 30 crashes it :(
		interval = self.BIOSENSOR_MAP[self.sensor]["sample_interval"]
		samplerate = self.BIOSENSOR_MAP[self.sensor]["sample_rate"]
		channels = [1]
		length = int(window * samplerate / 1000 * self.downsample)

		self.data_q = queue.Queue()

		print("Sample Rate found: ", samplerate)

		self.plotdata = np.zeros((length,len(channels)))
		print("plotdata shape found: ", self.plotdata.shape)
		self.fig, self.ax = plt.subplots(figsize=(8,4))

		self.lines = self.ax.plot(self.plotdata,color = (0,1,0.29))

		self.ax.set_title("Athelas: " + str(self.sensor) + " data")
		self.ax.set_facecolor((0,0,0))
		self.ax.set_yticks([0])
		self.ax.yaxis.grid(True)

		self.start_time = time.time()
		self.sub = rospy.Subscriber(f"biosensors/{topic}", biodatat, self.__bio_callback)

		self.ani = FuncAnimation(self.fig, plot_callback, fargs=(self,), interval=interval, blit=True)
		plt.show()

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
		datalist = data.data
		self.data_q.put(datalist)