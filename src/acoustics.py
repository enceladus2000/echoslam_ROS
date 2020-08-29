import numpy as np 
import pointhelper as ph
from scipy import signal

# All properties of air at 25°C and atm pressure
sound_speed = 343  # m/s
eta = 1.844E-5  # Dynamic viscosity in kg/m.s
rho = 1.1845  # Fluid density in kg/m^3
samplingRate = 44000 # Hz

# simple omnidirectional mic
class Mic:
	# position must be a np.array(2)
	def __init__(self, pos):
		self.pos = pos

	# convenient for debugging
	def __repr__(self):
		return 'Mic: pos = {p}'.format(p=self.pos)

	def simulate_waveform(self, src_pos, src_wave, sampling_rate, num_samples):
		pass

# class containing array of Mic[]
class MicArray:
	def __init__(self, array_size, radius, samping_rate=44100, num_samples=200):
		self.mics = []
		self.array_size = array_size
		self.sampling_rate = sampling_rate
		self.num_samples = num_samples

		# TODO: make this circular array instead
		# for x in np.linspace(-length/2, length/2, self.array_size):
		# 	self.mics.append(Mic((x, 0)))

	# iterable, must be able to be iterated, eg: for mic in micarray
	def __iter__(self):
		self.i = 0
		return self

	def __next__(self):
		if self.i < self.array_size:
			temp_mic = self.mics[self.i]
			self.i += 1
			return temp_mic
		else:
			raise StopIteration
	
	def __getitem__(self, index):
		return self.mics[index]
	
	def __len__(self):
		return self.array_size

class generateWaveform:
	'''
	Creates a sine waveform
	Parameters:
		freq (int): frequency of sine wave
		sampleSize (int): number of samples in waveform
		samplingRate (int): 
	Returns:
	Class instance with terms
		waveform (np.array): of length sampleSize
		amplitude
		phi - phase difference
		frequency
	'''
	def __init__(self,freq, sampleSize, samplingRate=44100):
		
		self.amp = 10 # Source wave amplitude
		# wave is of the form A0*sin(2*pi*v*t)
		# simple sin wave omnidirectional
		self.phi = 0
		t_range = np.linspace(0, sampleSize/samplingRate, num=sampleSize)
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
	# sample_delay = int(samplingRate * ph.distance(mic_pos, source_pos) / sound_speed)
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

def calcTimeDelay(og_wave, dly_wave, samplingRate):
	'''
	Calculates how much dly_wave is behind og_wave, using cross correlation(?)
		Parameters:
			og_wave (np.array): original wave
			dly_wave (np.array): delayed wave (whose delay we want to find)
			samplingRate (int): 
		Returns:
			delay (float): in seconds
	'''
	n=len(og_wave)
	corr = signal.correlate(dly_wave, og_wave, mode='same') / np.sqrt(signal.correlate(og_wave, og_wave, mode='same')[int(n/2)] * signal.correlate(dly_wave, dly_wave, mode='same')[int(n/2)])
	delay_arr = np.linspace(-n/samplingRate, n/samplingRate, len(corr))
	print(len(delay_arr))
	delay = delay_arr[np.argmax(corr)]
	print('dly_wave is ' + str(delay) + ' behind og_wave')

	return delay

def calcTOFs(og_wave, rec_waveforms):
	'''
	Performs calcTimeDelay() on each waveform in rec_waveforms
		Parameters:
			og_wave (np.array): original wave, as reference
			rec_waveforms (list of np.arrays): list of waveforms
			samplingRate (int): 
		Returns:
			TOFs (list of float): in seconds
	'''
	# loop calcTimeDelay(transmitted_wave, recwaveform[i])
	pass

# main function for testing only
# you can test your functions here
# if __name__ == '__main__':
# 	micposlist = [np.array((1, 2)), np.array((1,1.8)), np.array((1, 1.6))]
# 	sourcepos = np.array((1,2.5))
# 	waveforms = simWaveform(micposlist, sourcepos)
# 	import matplotlib.pyplot as plt 
# 	for waveform in waveforms:
# 		plt.plot(waveform)

# 	dly = calcTimeDelay(waveforms[0], waveforms[1])
# 	print('distance = ', dly*sound_speed)

# 	plt.show()