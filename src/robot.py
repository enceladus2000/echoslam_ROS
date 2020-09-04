from echoslam_ROS.msg import Bot
import rospy
import random 
import numpy as np
from pointhelper import *
from trilaterate import *

class Robot:
	msg = None
	topic_name = '/RF'
	bot_name = None

	mic_array = [[0,0]]		# array of mics' positions wrt center of bot
	transmitter_pos = [0,0]	# position of transmitter wrt center of bot

	def __init__(self, id=0, teamsize=1, pos=(0,0)):
		self.id = id
		self.teamsize = teamsize
		self.pos = np.array(pos, dtype=np.float32)
		self.radius = rospy.get_param('/radius')

	def getBotName(self):
		return 'bot' + str(self.id)

	# args are corners of rectangle in cartesian
	def initRandomPos(self, corner1, corner2):
		self.pos[0] = random.uniform(corner1[0], corner2[0])
		self.pos[1] = random.uniform(corner1[1], corner2[1])

	### 20/08/2020 : Aditya Bidwai
	def setPose(self, response_obj):
		self.pos[0] = response_obj.bot.x.data
		self.pos[1] = response_obj.bot.y.data
	###

	# compile bot info into a message
	# must call 
	def createMsg(self):
		self.msg = Bot()
		self.msg.id.data = self.id
		self.msg.x.data = self.pos[0]
		self.msg.y.data = self.pos[1]

	# create circular mic array
	def setMicArray(self, num_mics, radius):
		self.mic_array = []

		for i in range(num_mics):
			angle = 2*i*np.pi/num_mics
			pos = radius * np.array([np.cos(angle), np.sin(angle)])
			self.mic_array.append(pos)

	# source_pos is 2D np.array of global position of other bot's transmitter
	def getMicDOFs(self, source_pos):
		micDOFs = []
		for mic_pos in self.mic_array:
			thisDOF = distance(mic_pos+self.pos, source_pos)
			micDOFs.append((mic_pos, thisDOF))

		return micDOFs

	# returns 2D vector of relative position of other robot wrt this robot
	def trilaterate(self, micDOFs):
		est_rel_pos = geo_trilaterate(micDOFs)
		return est_rel_pos



