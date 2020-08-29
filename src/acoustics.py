import numpy as np 
import pointhelper as ph
from scipy import signal

# All properties of air at 25°C and atm pressure
sound_speed = 343  # m/s
eta = 1.844E-5  # Dynamic viscosity in kg/m.s
rho = 1.1845  # Fluid density in kg/m^3
sampling_rate = 44000 # Hz

class Waveform:
	# sine wave only
	def __init__(self, freq, phi=0, sampling_rate=44100, num_samples=200):
		self.amp = 10	# Source wave amplitude
		self.freq = freq
		self.phi = phi

		# wave is of the form A0*sin(2*pi*v*t)
		t_range = np.linspace(0, num_samples/sampling_rate, num=num_samples)
		self.wave = self.amp*np.sin(2*np.pi*self.freq * t_range + self.phi)

	# TODO: construcotr/classmethod for arbitrary signals?

# simple omnidirectional mic
class Mic:
	# position must be a np.array(2)
	def __init__(self, pos):
		self.pos = pos

	# convenient for debugging
	def __repr__(self):
		return 'Mic: pos = {p}'.format(p=self.pos)

	def simulate_waveform(self, src_pos, src_wave):
		# Distance between source and mic
		dist = ph.distance(self.pos, src_pos)

		# TODO @Ashutosh: Add proper delay, not phase delay.
		# Phase difference in recieved wave due to distance
		# source_wave.phi = (2*np.pi*source_wave.freq * dist) / sound_speed

		# Attenuation calculated by Stoke's law of sound attenuation
		# A = A0*e^(-alpha*d), alpha = 2*eta*v^2/3*rho*c^3
		alpha = (2*eta*(src_wave.freq**2)) / (3*rho*(sound_speed**3))
		src_wave.amp = src_wave.amp * np.exp(-alpha * dist)

		return src_wave

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

	def generateWaveforms(self, src_pos, src_wave):
		self.waveforms = []

		# TODO: should we have a check for src_wave.sampling_rate and num_samples?
		# or should we not have those fields in MicArray at all?
		for mic in self.mics:
			self.waveforms.append(mic.generateWaveform(src_pos, src_wave))
		return self.waveforms		

class generateWaveform:
	'''
	Creates a sine waveform
	Parameters:
		freq (int): frequency of sine wave
		num_samples (int): number of samples in waveform
		sampling_rate (int): 
	Returns:
	Class instance with terms
		waveform (np.array): of length num_samples
		amplitude
		phi - phase difference
		frequency
	'''
	def __init__(self,freq, num_samples, sampling_rate=44100):
		
		self.amp = 10 # Source wave amplitude
		# wave is of the form A0*sin(2*pi*v*t)
		# simple sin wave omnidirectional
		self.phi = 0
		t_range = np.linspace(0, num_samples/sampling_rate, num=num_samples)
		self.freq = freq
		self.wave = self.amp*np.sin(2*np.pi*self.freq * t_range + self.phi)


def simWaveform(mic_pos, source_pos, source_wave):
	'''
	Simulates propagation of audio thru air. Namely, applies appropriate time delay
	and attenuation to source_wave
		Parameters:
			mic_pos (np.array(2)): cartesian coords of mic (receiver)
			source_pos (np.array(2)): coords of source
			source_wave (class instance of generateWaveform): waveform that the source emits
		Returns:
			simulated_wave (np.array): output waveform from mic
	'''
	# sample_delay = int(sampling_rate * ph.distance(mic_pos, source_pos) / sound_speed)
	# Ashutosh's code

	# Distance between source and mic
	d =np.array(mic_pos) - np.array(source_pos)
	dist = np.sqrt(np.dot(d,d))

	# Phase difference in recieved wave due to distance
	source_wave.phi = (2*np.pi*source_wave.freq * dist) / sound_speed

	# Attenuation calculated by Stoke's law of sound attenuation
	# A = A0*e^(-alpha*d), alpha = 2*eta*v^2/3*rho*c^3
	alpha = (2*eta*(source_wave.freq**2)) / (3*rho*(sound_speed**3))
	source_wave.amp = source_wave.amp * np.exp(-alpha * dist)

	return source_wave

# def calcTimeDelay(og_wave, dly_wave, sampling_rate):
# 	'''
# 	Calculates how much dly_wave is behind og_wave, using cross correlation(?)
# 		Parameters:
# 			og_wave (np.array): original wave
# 			dly_wave (np.array): delayed wave (whose delay we want to find)
# 			sampling_rate (int): 
# 		Returns:
# 			delay (float): in seconds
# 	'''
# 	n=len(og_wave)
# 	corr = signal.correlate(dly_wave, og_wave, mode='same') / np.sqrt(signal.correlate(og_wave, og_wave, mode='same')[int(n/2)] * signal.correlate(dly_wave, dly_wave, mode='same')[int(n/2)])
# 	delay_arr = np.linspace(-n/sampling_rate, n/sampling_rate, len(corr))
# 	print(len(delay_arr))
# 	delay = delay_arr[np.argmax(corr)]
# 	print('dly_wave is ' + str(delay) + ' behind og_wave')

# 	return delay

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
	MicArray array = MicArray(8, 1)
	src_pos = np.array((2, 2))
	src_wave = Waveform(1000)
	