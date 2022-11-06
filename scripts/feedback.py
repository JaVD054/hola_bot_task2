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


######################## IMPORT MODULES ##########################

import numpy as np			# If you find it required
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2				# OpenCV Library
from cv2 import aruco
import math				# If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation

############################ GLOBALS #############################

aruco_publisher = rospy.Publisher('detected_aruco', Pose2D, queue_size=10)
aruco_msg = Pose2D()

# cv
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def callback(data):
	# Bridge is Used to Convert ROS Image message to OpenCV image
	br = CvBridge()
	rospy.loginfo("Image Received")
	get_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
	frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)
	# gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	marker_corners, marker_IDs, reject = aruco.detectMarkers(frame, marker_dict, parameters=param_markers)
	if marker_corners:
		for ids, corners in zip(marker_IDs, marker_corners):
			cv2.polylines(
				frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
			)
			# print(corners.astype(np.int32))
			corners = corners.reshape(4, 2)
			corners = corners.astype(int)
			top_left = corners[0].ravel()
			top_right = corners[1].ravel()
			bottom_right = corners[2].ravel()
			bottom_left = corners[3].ravel()
			# print(top_right,top_left,bottom_right,bottom_left)
			cv2.putText(frame, f"id: {ids[0]}", top_right, cv2.FONT_HERSHEY_PLAIN, 1.3, (200, 100, 0), 2, cv2.LINE_AA,)
			# print(ids, "  ", corners)

	d = (int((top_right[0]+bottom_left[0])/2), int((top_right[1]+bottom_left[1])/2))
	# d2 = (int((top_left[0]+bottom_right[0])/2), int((top_left[1]+bottom_right[1])/2))
	
	cv2.imshow("frame", frame)
	key = cv2.waitKey(1)
	aruco_msg.x = d[0]
	aruco_msg.y = d[1]
	aruco_msg.theta = math.atan((top_right[1]-top_left[1])/(top_right[0]-top_left[0]))
	aruco_publisher.publish(aruco_msg)
	print(aruco_msg)
	# cv2.imshow("image",frame)
	# cv2.waitKey(1)


	
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Use OpenCV to find ARUCO MARKER from the IMAGE
	#	-> You are allowed to use any other library for ARUCO detection, 
	#        but the code should be strictly written by your team and
	#	   your code should take image & publish coordinates on the topics as specified only.  
	#	-> Use basic high-school geometry of "TRAPEZOIDAL SHAPES" to find accurate marker coordinates & orientation :)
	#	-> Observe the accuracy of aruco detection & handle every possible corner cases to get maximum scores !

	############################################
      
def main():
	rospy.init_node('aruco_feedback_node')  
	rospy.Subscriber('overhead_cam/image_raw', Image, callback)
	rospy.spin()
  
if __name__ == '__main__':
  main()
