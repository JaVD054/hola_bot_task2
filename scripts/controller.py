#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user

from geometry_msgs.msg import Wrench	# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

#import pid

import numpy
import time
import math		# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles

################## GLOBAL VARIABLES ######################


PI = 3.14

x_goals = []
y_goals = []
theta_goals = []

right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

hola_theta = 0
hola_x = 0
hola_y = 0


##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def signal_handler(sig, frame):
	
	# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
	print('Clean-up !')
	cleanup()
	sys.exit(0)

def cleanup():
	r_force = Wrench()
	l_force = Wrench()
	f_force = Wrench()

	r_force.force.x = 0	
	r_force.force.y = 0
	r_force.force.z = 0
	r_force.torque.x = 0
	r_force.torque.y = 0
	r_force.torque.z = 0

	l_force.force.x = 0
	l_force.force.y = 0
	l_force.force.z = 0
	l_force.torque.x = 0
	l_force.torque.y = 0
	l_force.torque.z = 0

	f_force.force.x = 0
	f_force.force.y = 0
	f_force.force.z = 0
	f_force.torque.x = 0
	f_force.torque.y = 0
	f_force.torque.z = 0

	right_wheel_pub.publish(r_force)
	front_wheel_pub.publish(l_force)
	left_wheel_pub.publish(f_force)
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Not mandatory - but it is recommended to do some cleanup over here,
	#	   to make sure that your logic and the robot model behaves predictably in the next run.

	############################################
  
  
def task2_goals_Cb(msg):
	global x_goals, y_goals, theta_goals
	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()

	for waypoint_pose in msg.poses:
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)

def aruco_feedback_Cb(msg):
	global hola_theta, hola_x, hola_y
	hola_theta = msg.theta
	hola_x = msg.x
	hola_y = msg.y
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Receive & store the feedback / coordinates found by aruco detection logic.
	#	-> This feedback plays the same role as the 'Odometry' did in the previous task.

	############################################




def inverse_kinematics(w,v_x,v_y,kp):
	global right_wheel_pub, left_wheel_pub, front_wheel_pub
	h = numpy.array([[-0.17483,1,0],
					[-0.17483,-.5,-.86602540378],
					[-0.17483,-.5,.86602540378]])
	l = numpy.array([[w,],[v_x,],[v_y,]])
	forces =numpy.dot(1/.05,h@l)
	return [
		forces[0][0],forces[1][0],forces[2][0]
	]
	
	
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Use the target velocity you calculated for the robot in previous task, and
	#	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
	#	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
	############################################


def main():

	rospy.init_node('controller_node')

	signal.signal(signal.SIGINT, signal_handler)

	# NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	#	Use the below given topics to generate motion for the robot.
	right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
	front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
	left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

	rospy.Subscriber('detected_aruco',Pose2D,aruco_feedback_Cb)
	rospy.Subscriber('task2_goals',PoseArray,task2_goals_Cb)
	
	rate = rospy.Rate(100)

	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Make use of the logic you have developed in previous task to go-to-goal.
	#	-> Extend your logic to handle the feedback that is in terms of pixels.
	#	-> Tune your controller accordingly.
	# 	-> In this task you have to further implement (Inverse Kinematics!)
	#      find three omni-wheel velocities (v1, v2, v3) = left/right/center_wheel_force (assumption to simplify)
	#      given velocity of the chassis (Vx, Vy, W)
	r_force = Wrench()
	l_force = Wrench()
	f_force = Wrench()

	r_force.force.x = 0	
	r_force.force.y = 0
	r_force.force.z = 0
	r_force.torque.x = 0
	r_force.torque.y = 0
	r_force.torque.z = 0

	l_force.force.x = 0
	l_force.force.y = 0
	l_force.force.z = 0
	l_force.torque.x = 0
	l_force.torque.y = 0
	l_force.torque.z = 0

	f_force.force.x = 0
	f_force.force.y = 0
	f_force.force.z = 0
	f_force.torque.x = 0
	f_force.torque.y = 0
	f_force.torque.z = 0

	right_wheel_pub.publish(r_force)
	front_wheel_pub.publish(l_force)
	left_wheel_pub.publish(f_force)

		
	while not rospy.is_shutdown():
		
		f_F,r_F,l_F = inverse_kinematics(0,2,0,kp=5)
		print(f_F,r_F,l_F,sep='\n')
		r_force.force.x = r_F
		l_force.force.x = l_F
		f_force.force.x = f_F

		right_wheel_pub.publish(r_force)
		front_wheel_pub.publish(l_force)
		left_wheel_pub.publish(f_force)
		
		# Calculate Error from feedback

		# Change the frame by using Rotation Matrix (If you find it required)

		# Calculate the required velocity of bot for the next iteration(s)
		
		# Find the required force vectors for individual wheels from it.(Inverse Kinematics)

		# Apply appropriate force vectors

		# Modify the condition to Switch to Next goal (given position in pixels instead of meters)

		rate.sleep()

    ############################################

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

