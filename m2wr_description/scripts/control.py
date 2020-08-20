#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2

x = 0.0
y = 0.0
theta = 0.0

def newOdom(msg):

	global x
	global y
	global theta

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y

	rot_q = msg.pose.pose.orientation
	(roll,pitch,theta) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
	#print(theta)

rospy.init_node('speed_controller')

sub = rospy.Subscriber("/odom",Odometry,newOdom)
pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)

speed = Twist()

r = rospy.Rate(4)

goal = Point()
goal.x = 8
goal.y = 8

while not rospy.is_shutdown():
	inc_x = goal.x - x
	inc_y = goal.y - y

	angle_to_goal = atan2(inc_y,inc_x)

	if abs(angle_to_goal-theta) > 0.1:
		print(angle_to_goal)
		print(theta)
		speed.linear.x = 0.0
		speed.angular.z = 0.3

	else :

		speed.linear.x = 0.6
		speed.angular.z = 0.0

		if inc_x <0.1 and inc_y <0.1 :
         
				speed.linear.x = 0.0
				speed.angular.z = 0.0
				pub.publish(speed)
				break

	pub.publish(speed)

	r.sleep()
