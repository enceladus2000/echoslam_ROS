# hashbang required?
from echoslam.msg import Bot
import random 
import numpy as np

class Robot:
	# python recommendation for variables in class?
	msg = None
	topic_name = 'RF'
	bot_name = None

	def __init__(self, id=0, teamsize=1, pos=(0,0)):
		self.id = id
		self.teamsize = teamsize
		self.pos = np.array(pos, dtype=np.float32)

	def getBotName(self):
		return 'bot' + str(id)

	# args are corners of rectangle in cartesian
	def initRandomPos(self, corner1, corner2):
		self.pos[0] = random.uniform(corner1[0], corner2[0])
		self.pos[1] = random.uniform(corner1[1], corner2[1])

	def createMsg(self):
		self.msg = Bot()
		self.msg.id.data = self.id
		self.msg.x.data = self.pos[0]
		self.msg.y.data = self.pos[1]


