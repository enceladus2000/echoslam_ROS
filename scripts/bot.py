#! /usr/bin/env python

from echoslam.msg import Bot
import sys
import rospkg
import rospy
import getopt
from random import random

# import robot class from src folder
rospack = rospkg.RosPack()
path = rospack.get_path('echoslam_ROS')
sys.path.append(path)
import src
from src.robot import Robot

def callback(msg):
	if msg.id.data != robot.id:
		now = int(rospy.get_time())
		print('{}: Incoming message...'.format(now))
		print('bot_id:', msg.id.data)
		print('bot_pos: {x: .2f},{y: .2f}'.format(x=msg.x.data, y=msg.y.data))

	# check if bot that just transmitted is the one before
	if msg.id.data % robot.teamsize == robot.id - 1:
		print('My turn to transmit! Waiting for {:.2f}s...'.format(transmit_delay))
		rospy.sleep(transmit_delay)
		print('Transmitting...')
		pub.publish(robot.msg)

transmit_delay = 1.0	# time delay between transmissions, in seconds
robot = Robot()
sub = rospy.Subscriber(robot.topic_name, Bot, callback)
pub = rospy.Publisher(robot.topic_name, Bot, queue_size=3)

def main():
	# procure teamsize and id from param server
	try:
		robot.teamsize = rospy.get_param("/size")
		robot.id = rospy.get_param("id")
	except KeyError as error:
		print('The parameter {} was not found'.format(error))
		print('Exiting...')
		sys.exit(2)

	robot.initRandomPos((0, 0), (10, 10))
	robot.createMsg()
	rospy.init_node("dummy_node_name")		# required??
	print('Starting {bot}...'.format(bot=robot.getBotName()))

	# break the ice
	if robot.id == 1:
		pub.publish(robot.msg)

	rospy.spin()

	

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
