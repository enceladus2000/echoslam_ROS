#! /usr/bin/env python

from echoslam_ROS.msg import Bot #28/08/2020 : Aditya changed echoslam to echoslam_ROS in this line.
from echoslam_ROS.srv import *
from std_msgs.msg import Int32
import sys
import rospkg
import rospy
import getopt
import numpy as np 
from random import random

"""TODO:
1. add audio sim function placeholder
2. implement aadhar's time lag code
3. orientation?
4. mic array class?
5. Decide objects heirarchy lol
"""

# import robot class from src folder

###### 20/08/2020 : Aditya
def bot_server_client(request_obj):
	response_obj = bot_server_proxy(request_obj)
	return response_obj
######

rospack = rospkg.RosPack()
path = rospack.get_path('echoslam_ROS')
sys.path.append(path)
from src.robot import Robot
from src.acoustics import simReceivedWaveform, calcTOFs

# gets called once bot receives message on topic
def callback(msg):
	# ignore messages sent by itself
	if msg.id.data != robot.id:
		print(robot.bot_name + ' has received a message...')
		print(msg)
		source_pos = np.array((msg.x.data, msg.y.data))
		micDOFs = robot.getMicDOFs(source_pos)
		rel_pos = robot.trilaterate(micDOFs)
		print('Source position: ', source_pos)
		print('Estimated relative position:', rel_pos)
		print('Actual relative position:', source_pos-robot.pos)
		# source_pos = np.array((msg.x.data, msg.y.data))
		# rec_waveforms = simReceivedWaveform(robot.getMicPositions, souce_pos)
		# # TOFs = calcTOFs(rec_waveforms)
		# robot.trilaterate(TOFs)

		# print results


	# check if bot that just transmitted is the one before
	if msg.id.data % robot.teamsize == robot.id - 1:
		rospy.sleep(transmit_delay)
		pub.publish(robot.msg)


robot = Robot()		
rospy.wait_for_service('/bot_service')
bot_server_proxy = rospy.ServiceProxy('/bot_service', BotService)
sub = rospy.Subscriber(robot.topic_name, Bot, callback)
pub = rospy.Publisher(robot.topic_name, Bot, queue_size=3)


transmit_delay = 1.0	# time delay between transmissions, in seconds
np.set_printoptions(precision=4)

def main():
	# procure teamsize and id from param server
	try:
		robot.teamsize = rospy.get_param("/size")
		robot.id = rospy.get_param("id")
		robot.bot_name = 'bot' + str(robot.id)
	except KeyError as error:
		print('The parameter {} was not found'.format(error))
		print('Exiting...')
		sys.exit(2)

	#robot.initRandomPos((0, 0), (5, 5))		# spawn bot in some random position
	robot.setPose(bot_server_client(BotServiceRequest(Int32(robot.id))))
	robot.setMicArray(6, .1)
	robot.createMsg()							# must call before pub.publish(robot.msg)

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
