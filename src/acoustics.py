import numpy as np 
import pointhelper as ph
from scipy import signal

sound_speed = 343  # m/s
samplingRate = 44000

def simWaveform(rec_pos_list, source_pos):
	# create simple sine wave (continuous)
	sampleSize = 200
	freq = 1000
	t = np.linspace(0, sampleSize/samplingRate, sampleSize)
	og_wave = np.sin(2*np.pi*freq*t)

	# append it to appropriate delay for each rec_pos
	sim_waveforms = []
	for rec_pos in rec_pos_list:
		sample_delay = int(samplingRate * ph.distance(rec_pos, source_pos) / sound_speed)
		wave = np.zeros(sampleSize)
		wave[sample_delay:] = og_wave[:-sample_delay]
		sim_waveforms.append(wave)
	
	return sim_waveforms

def calcTimeDelay(y1, y2):
	n=len(y1)
	corr = signal.correlate(y2, y1, mode='same') / np.sqrt(signal.correlate(y1, y1, mode='same')[int(n/2)] * signal.correlate(y2, y2, mode='same')[int(n/2)])
	delay_arr = np.linspace(-n/samplingRate, n/samplingRate, len(corr))
	print(len(delay_arr))
	delay = delay_arr[np.argmax(corr)]
	print('y2 is ' + str(delay) + ' behind y1')

	return delay

def calcTOFs(rec_waveforms):
	# add aadhar's code for time_lag
	# loop calcTimeDelay(transmitted_wave, recwaveform[i])
	pass

# main function for testing only
if __name__ == '__main__':
	micposlist = [np.array((1, 2)), np.array((1,1.8)), np.array((1, 1.6))]
	sourcepos = np.array((1,2.5))
	waveforms = simWaveform(micposlist, sourcepos)
	import matplotlib.pyplot as plt 
	for waveform in waveforms:
		plt.plot(waveform)

	dly = calcTimeDelay(waveforms[0], waveforms[1])
	print('distance = ', dly*sound_speed)

	plt.show()