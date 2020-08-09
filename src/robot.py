from echoslam.msg import Bot
import random 
import numpy as np

class Robot:
	msg = None
	topic_name = '/RF'
	bot_name = None

	# array of mics' positions wrt center of bot
	mic_array = [[0,0]]
	# position of transmitter wrt center of bot
	transmitter_pos = [0,0]

	def __init__(self, id=0, teamsize=1, pos=(0,0)):
		self.id = id
		self.teamsize = teamsize
		self.pos = np.array(pos, dtype=np.float32)

	def getBotName(self):
		return 'bot' + str(self.id)

	# args are corners of rectangle in cartesian
	def initRandomPos(self, corner1, corner2):
		self.pos[0] = random.uniform(corner1[0], corner2[0])
		self.pos[1] = random.uniform(corner1[1], corner2[1])

	# compile bot info into a message
	# must call 
	def createMsg(self):
		self.msg = Bot()
		self.msg.id.data = self.id
		self.msg.x.data = self.pos[0]
		self.msg.y.data = self.pos[1]

	# def setMicArray(self, num_mics, radius):
	# 	self.mic_array = []

	# 	for i in range(num_mics):
	# 		angle = 2*i*np.pi/num_mics
	# 		pos = radius * np.array([np.cos(angle), np.sin(angle)])
	# 		self.mic_array.append(pos)

