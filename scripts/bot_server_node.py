#! /usr/bin/env python 

import rospy  
from echoslam_ROS.srv import BotService, BotServiceResponse

response_obj = BotServiceResponse()

def bot_server(bot_id):
	response_obj.x.data = bot_id.data
	response_obj.y.data = bot_id.data
	response_obj.id.data = bot_id.data
	return response_obj

rospy.init_node("bot_server_node")
bot_service = rospy.Service('/bot_service', BotService, bot_server)
print("BOT-SERVER INITIALIZED")
rospy.spin()