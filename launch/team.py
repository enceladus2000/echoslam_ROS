#! /usr/bin/env python
import roslaunch 
import rospy
import sys
import rospkg

rospack = rospkg.RosPack()
path = rospack.get_path('echoslam_ROS')

rospy.init_node("launcher", anonymous = True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
cli_args = [path + "/launch/team.launch", "teamsize:=4"]
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
parent.start()
rospy.spin()