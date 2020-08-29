#! /usr/bin/env python

from echoslam.msg import Bot
import sys
import rospkg
import rospy
import getopt
import numpy as np 
from random import random

"""TODO:

"""

# import robot class from src folder
rospack = rospkg.RosPack()
path = rospack.get_path('echoslam_ROS')
sys.path.append(path)
from src.robot import Robot
from src.acoustics import simReceivedWaveform, calcTOFs

# gets called once bot receives message on topic
def callback(msg):
	# ignore messages sent by itself
	if msg.id.data != robot.id:
		source_pos = np.array((msg.x.data, msg.y.data))
		
		print('Estimated Relative Position:', est_rel_pos)
		print('Actual Relative Position:', source_pos-robot.pos)

	# check if bot that just transmitted is the one before
	if msg.id.data % robot.teamsize == robot.id - 1:
		rospy.sleep(transmit_delay)
		pub.publish(robot.msg)


robot = Robot(radius=0.2)			# create Robot object
sub = rospy.Subscriber(robot.topic_name, Bot, callback)
pub = rospy.Publisher(robot.topic_name, Bot, queue_size=3)

transmit_delay = 1.0	# time delay between transmissions, in seconds
np.set_printoptions(precision=4)

def main():
	# procure teamsize and id from param server
	try:
		robot.teamsize = rospy.get_param("/size")
		robot.id = rospy.get_param("id")
	except KeyError as error:
		print('The parameter {} was not found'.format(error))
		print('Exiting...')
		sys.exit(2)

	robot.init_random_pos((0, 0), (5, 5))		# spawn bot in some random position
	robot.init_mic_array(num_mics=6, radius=0.1)
	robot.init_msg()							# must call before pub.publish(robot.msg)

	# node name is determined by the launch file
	# but init_node still needs to be called			
	rospy.init_node(robot.bot_name)	
	print('Starting {}...'.format(robot.bot_name))

	# bot1 will be initialised last, and it will start the msg chain
	if robot.id == 1:
		print('Bot1 initialising conversation...')
		pub.publish(robot.msg)

	rospy.spin()

def printMsg(msg):
	# print received message with epoch
	# now = int(rospy.get_time())
	print('Incoming message...')
	print('bot_id:', msg.id.data)
	print('bot_pos: {x: .2f},{y: .2f}'.format(x=msg.x.data, y=msg.y.data))

# deprecated
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
