import numpy as np 
import pointhelper as ph
from scipy import signal

sound_speed = 343  # m/s
samplingRate = 44000

def generateWaveform(freq, sampleSize, samplingRate=44100):
	'''
	Creates a sine waveform
	Parameters:
		freq (int): frequency of sine wave
		sampleSize (int): number of samples in waveform
		samplingRate (int): 
	Returns:
		waveform (np.array): of length sampleSize
	'''
	pass

def simWaveform(mic_pos, source_pos, source_wave):
	'''
	Simulates propagation of audio thru air. Namely, applies appropriate time delay
	and attenuation to source_wave
		Parameters:
			mic_pos (np.array(2)): cartesian coords of mic (receiver)
			source_pos (np.array(2)): coords of source
			source_wave (np.array): waveform that the source emits
		Returns:
			simulated_wave (np.array): output waveform from mic
	'''
	# sample_delay = int(samplingRate * ph.distance(mic_pos, source_pos) / sound_speed)
	
	# Ashutos's code
	pass

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