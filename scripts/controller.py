#!/usr/bin/env python3

import time
import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user

from geometry_msgs.msg import Wrench	# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

#import pid

import numpy
from time import sleep
import math		# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles



rospy.init_node('controller', anonymous=True)

# Global variables

hola_x = 0
hola_y = 0
hola_theta = 0

x_goals = [350,50,50,350,250]
y_goals = [350,350,50,50,250]
theta_goals = [0.785, 2.335, -3, 3, 0]

right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)
rate = rospy.Rate(100)



def signal_handler(sig, frame):
	print('Clean-up !')
	cleanup()
	sys.exit(0)


def cleanup():
	r_force = Wrench()
	l_force = Wrench()
	f_force = Wrench()

	r_force.force.x = 0	
	l_force.force.x = 0
	f_force.force.x = 0
	

	right_wheel_pub.publish(r_force)
	front_wheel_pub.publish(l_force)
	left_wheel_pub.publish(f_force)


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


def euclidean_distance(x1, y1, x2, y2):
	return math.sqrt((x1-x2)**2 + (y1-y2)**2)	


def steering_angle(x, y):
	global hola_x, hola_y, hola_theta
	return math.atan2(y-hola_y, x-hola_x)


def linear_vel_x(goal_x, goal_y, constant=2):
    global hola_x, hola_y, hola_theta
    return constant * math.cos(steering_angle(goal_x, goal_y)-hola_theta)

def linear_vel_y(goal_x, goal_y, constant=2):
    global hola_x, hola_y, hola_theta
    return constant * math.sin(steering_angle(goal_x, goal_y)-hola_theta)


def angular_vel(goal_theta, constant=3):
    return constant * (goal_theta-hola_theta)


def PID(Kp, Ki, Kd, MV_bar=0):
    e_prev = 0
    t_prev = -100
    I = 0
    
    MV = MV_bar
    
    while True:
        PV, SP = yield MV
        
        e =  PV - SP
        
        P = Kp*e
        I = I + Ki*e
        D = Kd*(e - e_prev)
        
        MV = MV_bar + P + I + D

        e_prev = e
    


def angle_range(theta):
    theta = theta%(2*math.pi) 
    if theta > math.pi or theta < -math.pi:
        theta = theta - 2*math.pi if theta > 0 else theta + 2*math.pi
    return theta

def inverse_kinematics(v_x, v_y, w,kp=1):
	global hola_x, hola_y, hola_theta
	h = numpy.array([[-0.17483,1,0],
					[-0.17483,-.5,-.86602540378],
					[-0.17483,-.5,.86602540378]]
					)
	vel_s = numpy.array([[w],[v_x],[v_y]])
	forces = h@vel_s
	return kp*forces
	
# The function moves the robot to the goal position and orientation
def move2goal(x_goal, y_goal, theta_goal):
	global hola_x, hola_y, hola_theta

	theta_goal = angle_range(theta_goal)
	rospy.loginfo("Goal: x: %d, y: %d, theta: %f", x_goal, y_goal, theta_goal)
	rospy.loginfo("Current: x: %d, y: %d, theta: %f", hola_x, hola_y, hola_theta)

	distance_tolerance = 0.1
	angle_tolerance = 0.01

	f_force = Wrench()
	r_force = Wrench()
	l_force = Wrench()

	pid = PID(5.6, 0.00001,0.00001)
	pid.send(None)
	error =euclidean_distance(x_goal, y_goal,hola_x,hola_y) 
	while error>= distance_tolerance or abs(hola_theta-theta_goal) > angle_tolerance:
		# print("x: ", hola_x, "y: ", hola_y, "theta: ", hola_theta)
		forces = inverse_kinematics(linear_vel_x(x_goal, y_goal,error/5), linear_vel_y(x_goal, y_goal,error/5), pid.send([hola_theta,theta_goal]),10)
		f_force.force.x = forces[1][0]
		r_force.force.x = forces[2][0]
		l_force.force.x = forces[0][0]

		right_wheel_pub.publish(r_force)
		front_wheel_pub.publish(l_force)
		left_wheel_pub.publish(f_force)
		error =euclidean_distance(x_goal, y_goal,hola_x,hola_y)
		rate.sleep()
	print( hola_x, hola_y, hola_theta)
	print("Goal reached")
	
	cleanup()
	sleep(3)


def main():
	global hola_x, hola_y, hola_theta, x_goals, y_goals, theta_goals

	rospy.Subscriber("/detected_aruco", Pose2D, aruco_feedback_Cb)
	rospy.Subscriber("/task2_goals", PoseArray, task2_goals_Cb)

	signal.signal(signal.SIGINT, signal_handler)

	while not rospy.is_shutdown():
		for x_goal, y_goal, theta_goal in zip(x_goals, y_goals, theta_goals):
			print(x_goal, y_goal, theta_goal)
			move2goal(x_goal, y_goal, theta_goal)
		x_goals.clear()
		y_goals.clear()
		theta_goals.clear()
		rate.sleep()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass


