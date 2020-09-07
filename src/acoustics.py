import numpy as np 
import pointhelper as ph
# from scipy import signal

# All properties of air at 25C and atm pressure
sound_speed = 343  			# m/s
eta = 1.844E-5  			# Dynamic viscosity in kg/m.s
rho = 1.1845  				# Fluid density in kg/m^3
sampling_rate = 44000 		# Hz

class Waveform:
	def __init__(self, freq, amp=1.0, phi=0, sampling_rate=44100, num_samples=200):
		'''
		Creates a sine waveform, typically representing the originally transmitted wave
			Parameters:
				freq (int): frequency of sine wave
				amp (float): amplitude of wave.
				phi (float): phase offset (rads)
				num_samples (int): number of samples in waveform
				sampling_rate (int): 
			Returns:
			Class instance with terms
				wave (np.array(num_samples)): np.array of waveform
		'''
		self.amp = amp	# Source wave amplitude
		self.freq = freq
		self.phi = phi
		self.sampling_rate = sampling_rate
		self.num_samples = num_samples

		# wave is of the form A0*sin(2*pi*v*t)
		t_range = np.linspace(0, num_samples/sampling_rate, num=num_samples)
		self.wave = self.amp*np.sin(2*np.pi*self.freq * t_range + self.phi)

	# DONE: construcotr/classmethod for arbitrary signals?
	@classmethod
	def from_array(cls, array, sampling_rate):
		# check array dimensions
		cls.wave = array
		cls.sampling_rate = sampling_rate
		cls.num_samples = len(array)
		cls.freq = None 		# add fft here?

		return cls

	def calcTimeDelay(self, og_wave, dly_wave, sampling_rate):
		'''
		Calculates how much dly_wave is behind og_wave, using cross correlation(?)
			Parameters:
				og_wave (np.array): original wave
				dly_wave (np.array): delayed wave (whose delay we want to find)
				sampling_rate (int): 
			Returns:
				delay (float): in seconds
		'''
		n=len(og_wave)
		corr = signal.correlate(dly_wave, og_wave, mode='same') / np.sqrt(signal.correlate(og_wave, og_wave, mode='same')[int(n/2)] * signal.correlate(dly_wave, dly_wave, mode='same')[int(n/2)])
		delay_arr = np.linspace(-n/sampling_rate, n/sampling_rate, len(corr))
		print(len(delay_arr))
		delay = delay_arr[np.argmax(corr)]
		print('dly_wave is ' + str(delay) + ' behind og_wave')

		return delay

# simple omnidirectional mic
class Mic:
	# position must be a np.array(2)
	def __init__(self, pos):
		self.pos = pos

	# convenient for debugging
	def __repr__(self):
		return 'Mic: pos = {p}'.format(p=self.pos)

	def simulate_waveform(self, src_pos, src_wave):
		"""
		Simulates the output of a mic
			Parameters:
				src_pos (np.array(2)): absolute coordinates of source
				src_wave (Waveform): waveform that the source emits
			Returns:
				sim_wave (Waveform): delayed, attenuated src_wave
		"""

		dist = ph.distance(self.pos, src_pos)		# Distance between source and mic
		MAX_DIST = 10 								# max possible distance of src (m)
		wave_sr = src_wave.sampling_rate			# aliases for src_wave
		wave_ns = src_wave.num_samples
		
		sim_ns = int(wave_ns + wave_sr*MAX_DIST/sound_speed)	# number of samples in simulated wave
		sim_wave_arr = np.zeros(sim_ns)

		# Attenuate src_wave, alculated by Stoke's law of sound attenuation
		# A = A0*e^(-alpha*d), alpha = 2*eta*v^2/3*rho*c^3
		alpha = (2*eta*(src_wave.freq**2)) / (3*rho*(sound_speed**3))
		atten_wave = np.exp(-alpha * dist) * src_wave.wave

		dly_ns = int(wave_sr*dist/sound_speed)			# time delay in number of samples
		sim_wave_arr[ dly_ns+1 : dly_ns+1+wave_ns ] = atten_wave
		
		# import matplotlib.pyplot as plt
		# plt.plot(sim_wave_arr)
		# plt.grid(True)
		# plt.show()

		# TODO: find better, less jugaadi way to do this!
		sim_wave = Waveform(freq=src_wave.freq, sampling_rate=wave_sr, num_samples=sim_ns)
		sim_wave.wave = sim_wave_arr
		return sim_wave

# class containing array of Mic[]
class MicArray:
	# inits circular mic array
	def __init__(self, num_mics, radius, samping_rate=44100, num_samples=200, array_pos=(0,0)):
		self.num_mics = num_mics
		self.radius = radius					# radius of mic array
		self.sampling_rate = sampling_rate		
		self.num_samples = num_samples			# number of samples in waveform
		self.array_pos = np.array(array_pos)	# pos of centre of circular array

		self.mics = []						# list of Mic objects, with absolute positions
		# create circular array of mics
		for i in range(num_mics):
			angle = 2*i*np.pi/num_mics
			rel_pos = radius * np.array([np.cos(angle), np.sin(angle)])
			mic = Mic(self.array_pos + rel_pos)
			self.mics.append(mic)

	# iterable, must be able to be iterated, eg: for mic in micarray
	def __iter__(self):
		self.i = 0
		return self

	# returns next mic object
	def __next__(self):
		if self.i < self.num_mics:
			temp_mic = self.mics[self.i]
			self.i += 1
			return temp_mic
		else:
			raise StopIteration
	
	# returns current mic object
	def __getitem__(self, index):
		return self.mics[index]
	
	def __len__(self):
		return self.num_mics

	def simulate_waveforms(self, src_pos, src_wave):
		self.waveforms = []

		# TODO: should we have a check for src_wave.sampling_rate and num_samples?
		# or should we not have those fields in MicArray at all?
		for mic in self.mics:
			self.waveforms.append(mic.simulate_waveform(src_pos, src_wave))
		return self.waveforms		

# def calcTOFs(og_wave, rec_waveforms):
# 	'''
# 	Performs calcTimeDelay() on each waveform in rec_waveforms
# 		Parameters:
# 			og_wave (np.array): original wave, as reference
# 			rec_waveforms (list of np.arrays): list of waveforms
# 			sampling_rate (int): 
# 		Returns:
# 			TOFs (list of float): in seconds
# 	'''
# 	# loop calcTimeDelay(transmitted_wave, recwaveform[i])
# 	pass

# main function for testing only
# you can test your functions here
if __name__ == '__main__':
	array = MicArray(4, 1)
	src_pos = np.array((4, 3))
	src_wave = Waveform(1000)

	array.simulate_waveforms(src_pos, src_wave)
	
	import matplotlib.pyplot as plt

	# TODO: show subplots
	for waveform in array.waveforms:
		plt.plot(waveform.wave)
		print(id(waveform))
	plt.show()
	