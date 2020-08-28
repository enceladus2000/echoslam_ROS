#! /usr/bin/env python 

import rospy  
from echoslam_ROS.srv import BotService, BotServiceResponse

response_obj = BotServiceResponse()

def bot_server(request_obj):
	response_obj.bot.x.data = request_obj.id.data
	response_obj.bot.y.data = request_obj.id.data
	response_obj.bot.id.data = request_obj.id.data
	return response_obj

rospy.init_node("bot_server_node")
bot_service = rospy.Service('/bot_service', BotService, bot_server)
print("#######################")
print("BOT-SERVER INITIALIZED")
print("#######################")
rospy.spin()