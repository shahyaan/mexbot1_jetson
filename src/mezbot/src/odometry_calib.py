#!/usr/bin/env python


import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
from math import atan2

#set robot origin as starting point
x = 0.0
y = 0.0
yaw = 0.0

# define cmd_handler
def wheelOdom_callback(msg):
	global x
	global y
	global yaw

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	rot_quat = msg.pose.pose.orientation
	(roll, pitch, yaw) =  euler_from_quaternion([rot_quat.x, rot_quat.y, rot_quat.z, rot_quat.w])

#initialize node
rospy.init_node("odometry_calib")

#subscribe to /wheel_odom topic
rospy.Subscriber("/wheel_odom", Odometry, wheelOdom_callback)

#create publisher to publish
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

speed = Twist()

#set goal
goal = Point()
goal.x = 5.0 # change only this to calibrate linear movement back and forth 
goal.y = 0.0 # change this to calibrate turns


rate = rospy.Rate(10)

while not rospy.is_shutdown():
	error_x = goal.x - x
	error_y = goal.y - y
	angle_to_goal = atan2(error_y, error_x)

	# if you're within a certain threshold of desired position stop
	if ((error_x < 0.05) and (error_y < 0.05)):
		speed.linear.x = 0 
		speed.angular.z = 0
		rate.sleep()

#	#rotate in place before moving forward 
#	if abs(angle_to_goal - yaw) > 0.1:
#		speed.linear.x = 0.0
#		speed.angular.z = 1.5
	else:
		speed.linear.x = 0.5
		speed.angular.z = 0.0

	pub.publish(speed)
	rate.sleep()



