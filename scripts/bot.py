#! /usr/bin/env python

from echoslam.msg import Bot
from robot import Robot
import rospy
import sys
import getopt
import json
from random import random

def callback(msg):
	print('Incoming message...')
	print('bot_id:', msg.id.data)
	print('bot_pos: {x: .2f},{y: .2f}'.format(x=msg.x.data, y=msg.y.data))

robot = Robot()
sub = rospy.Subscriber(robot.topic_name, Bot, callback)
pub = rospy.Publisher(robot.topic_name, Bot, queue_size=1)

def main():
	try:
		robot.id, robot.teamsize = cl_args()
	except ValueError as error:
		print(error)
		print('Exiting...')
		sys.exit(2)

	robot.initRandomPos((0, 0), (10, 10))
	robot.createMsg()
	rospy.init_node('bot')

	


def cl_args():
	if len(sys.argv) <=1:
		raise ValueError('Please enter id and teamsize.')
		
	try:
		optlist, _ = getopt.getopt(sys.argv[1:], "i:n:", ['id=','teamsize=']) 
	except getopt.GetoptError:
		raise ValueError('Invalid arguments!')

	bot_id = None
	teamsize = None
	for opt, arg in optlist:
		if opt in ('-i', '--id'):
			bot_id = int(arg)
		elif opt in ('-n', '--teamsize'):
			teamsize = int(arg)

	if bot_id is None or teamsize is None:
		raise ValueError('Please enter both id and teamsize!')

	return bot_id, teamsize

if __name__ == '__main__':
	main()