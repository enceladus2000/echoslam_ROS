from echoslam.msg import Bot
import random 
import numpy as np
from pointhelper import *
from trilaterate import *
import acoustics as ac 

class Robot:
	msg = None
	topic_name = '/RF'

	transmitter_pos = [0,0]	# position of transmitter wrt center of bot
	transmitted_wave = None
	
	def __init__(self, id=0, teamsize=1, radius=1, pos=(0,0)):
		self.id = id
		self.teamsize = teamsize		# number of bots in a team
		self.bot_radius = radius		# radius of the circular bot
		self.pos = np.array(pos, dtype=np.float32)	# abs position of bot

	def init_mic_array(self, num_mics, radius=None, sampling_rate=44100, num_samples=200):
		if radius is None:
			marray_radius = self.bot_radius
		else:
			marray_radius = radius
		
		self.mic_array = ac.MicArray(num_mics, marray_radius,
								sampling_rate, num_samples, self.pos)

	def get_bot_name(self):
		return 'bot' + str(self.id)

	# args are corners of rectangle in cartesian
	def init_random_pos(self, corner1, corner2):
		self.pos[0] = random.uniform(corner1[0], corner2[0])
		self.pos[1] = random.uniform(corner1[1], corner2[1])

	# compile bot info into a message
	# must call 
	def init_msg(self):
		self.msg = Bot()
		self.msg.id.data = self.id
		self.msg.x.data = self.pos[0]
		self.msg.y.data = self.pos[1]

	def create_transmitted_wave(self, w_freq):
		self.transmitted_wave = ac.Waveform(
				freq=w_freq,
				sampling_rate=self.mic_array.sampling_rate,
				num_samples=self.mic_array.num_samples
			)
		return self.transmitted_wave
	
	def get_transmitted_wave(self):
		if self.transmitted_wave is None:
			self.create_transmitted_wave(1000)
		return self.transmitted_wave

	def record_waveforms(self, source_pos):
		tm_wave = self.get_transmitted_wave()
		self.mic_array.simulate_waveforms(source_pos, tm_wave)
		
		return self.mic_array.waveforms

	# # source_pos is 2D np.array of global position of other bot's transmitter
	# def getMicDOFs(self, source_pos):
	# 	micDOFs = []
	# 	for mic_pos in self.mic_array:
	# 		thisDOF = distance(mic_pos+self.pos, source_pos)
	# 		micDOFs.append((mic_pos, thisDOF))

	# 	return micDOFs

	# returns 2D vector of relative position of other robot wrt this robot
	def trilaterate(self, micDOFs):
		est_rel_pos = geo_trilaterate(micDOFs)
		return est_rel_pos
	



