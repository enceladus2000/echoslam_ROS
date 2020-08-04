import subprocess

"""TODO
1. Open each bot process in new terminal?
2. cl option for kill all bots?
3. pipe output from each bot script here and display nicely (maybe colored)
4. input cmd for killing/exiting
"""


# input teamsize from user

# initialise bot from ids n-1, n-2... 0
teamsize = input('Enter teamsize (number of bots)')

for i in reverse(range(teamsize)):
	args = '--id={id} --teamsize={ts}'.format(id=i, ts=teamsize)
	p = subprocess.Popen(['bot.py', 'test.py'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.PIPE)
	run bot.py with argg
