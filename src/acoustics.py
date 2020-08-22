import numpy as np 
import pointhelper as ph

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

def calcTOFs(rec_waveforms):
	# add aadhar's code for time_lag
	pass

# main function for testing only
if __name__ == '__main__':
	micposlist = [np.array((1, 2)), np.array((1,1.8)), np.array((1, 1.6))]
	sourcepos = np.array((1,2.2))
	waveforms = simWaveform(micposlist, sourcepos)
	import matplotlib.pyplot as plt 
	for waveform in waveforms:
		plt.plot(waveform)

	plt.show()