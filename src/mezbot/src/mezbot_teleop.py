#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys

twist = Twist()

def values():
	print ('w for forward, s for reverse, k for turning left, l for turning right and . to exit)' + '\n')
	s = raw_input(':- ')
	if s[0] == 'w':
		twist.linear.x = 0.3
		twist.angular.z = 0.0
		twist.linear.y = 0.0
	elif s[0] == 's':
		twist.linear.x = -0.3
		twist.angular.z = 0.0
		twist.linear.y = 0.0
	elif s[0] == 'k':
		twist.angular.z = 1.5
		twist.linear.x = twist.linear.y = 0.0
	elif s[0] == 'l':
		twist.angular.z = -1.5
		twist.linear.x = twist.linear.y = 0.0
	elif s[0] == '.':
		twist.angular.z = twist.linear.x = twist.linear.y = 0.0
		sys.exit()
	else:
		twist.linear.x = twist.linear.y = twist.angular.z = 0.0
		print ('Wrong command entered \n')

	return twist

def keyboard():
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	rospy.init_node('mezbot_teleop', anonymous=True)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		twist = values()
		pub.publish(twist)
		rate.sleep()

if __name__=='__main__':
	try:
		keyboard()
	except rospy.ROSInterruptException:
		pass
