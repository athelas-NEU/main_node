#!/usr/bin/env python3

from main_node.srv import GetKeypoint
from main_node.msg import biodatat
from std_msgs.msg import Int32, Float32, Bool, Int16, Float32MultiArray
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
		"pulse": {"keypoint": "hand", "topic": "heart", "distance": 20, "sample_rate":20.0, "sample_interval":125},
		"o2": {"keypoint": "hand", "topic": "spo2", "distance": 20, "sample_rate":20.0, "sample_interval":125},
		"temp": {"keypoint": "forehead", "topic": "temp", "distance": 1, "sample_rate":20.0, "sample_interval":125},
		"stethoscope": {"keypoint": "chest", "topic": "/biosensors/stethoscope", "distance": 0,"sample_rate": 44100.0, "sample_interval":200},
	}

	AXES_SETTINGS = {
		"pulse": {"ylim": [40, 200], "color": (0.9, 0.5, 0.2), "ylabel": "Average Pulse"},
		"o2": {"ylim": [90, 101], "color": (0, 0.1, 0.8), "ylabel": "O2 %"},
		"temp": {"ylim": [70, 105], "color":(0.7, 0, 0), "ylabel": "Temperature"},
		"stethoscope": {"ylim": [-0.4, 0.4], "color":(0, 1, 0.29), "ylabel": "Amplitude"}
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

		# biosensor = "stethoscope"
		# print("DEBUG: going to biosensor state")
		# return BioData(biosensor)

		biosensor = input("Enter the biosensor name: ")
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
			if self.location == "hand":
				self.pub_y.publish(-10)
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
			if data.data <= self.distance:
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

	state.ax.autoscale_view(scalex=False, scaley=True)

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
		self.start_time = time.time()
		self.sub = rospy.Subscriber(f"{topic}", Float32MultiArray, self.__bio_callback)

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

		self.lines = self.ax.plot(self.plotdata, color = self.AXES_SETTINGS[self.sensor]["color"])

		self.ax.set_title("Athelas: " + str(self.sensor) + " data")

		self.ax.set_facecolor((0,0,0))
		# self.ax.set_yticks([0])
		self.ax.set_ylabel(self.AXES_SETTINGS[self.sensor]["ylabel"])
		self.ax.tick_params(axis=u'y', which=u'major',length=0)

		self.ax.set_ylim(self.AXES_SETTINGS[self.sensor]["ylim"])

		self.ani = FuncAnimation(self.fig, plot_callback, fargs=(self,), interval=interval, blit=True)
		plt.show() ## is blocking, state should end after

		print("finished state init")

	def execute(self):
		super().execute()
		if time.time() - self.start_time > 15:
			self.sub.unregister()
			plt.close(self.fig)
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
		datalist = data.data
		self.data_q.put(datalist)
