// runs on the Jetson (Robot host) and subscribes to the '/mezbot_velocity_controller/odom' topic published by
// the arduino node (mezbot_control.ino)to convert the odometry message published by arduino (i.e., motor speeds) to a 2D odom
// message and a transform from "odom" coordinate frame to a "base_link" or "base_footprint" coordinate frame
// usable by the ROS navigation stack.

// Shahyaan Desai, Mezmeriz inc.
// July 2018

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

double gVel_Left = 0.0;
double gVel_Right = 0.0;

//subscribe to odom topic published by arduino which publishes left and right wheel velocities.  
void sub_callback(const geometry_msgs::Twist::ConstPtr &twist_msg){
	gVel_Left = twist_msg->linear.x;
	gVel_Right = twist_msg->linear.y;
	}

int main(int argc, char** argv){
	ros::init(argc, argv, "mezbot_odom_node");
	ros::NodeHandle _nh;
	geometry_msgs::TransformStamped odom_trans;					//instance a stamped transform message
	nav_msgs::Odometry odom;							//instance an odometry message
	geometry_msgs::Quaternion odom_quat;						//since odom os 6dof we need variable to create quaternion from yaw
	
	
	//subscribe to the odom message being published by the arduino
	ros::Subscriber odom_sub = _nh.subscribe("/mezbot_velocity_controller/odom", 50, sub_callback);
	
	//create a ros publisher object and transform broadcaster object to send messages to ros
	ros::Publisher odom_pub = _nh.advertise<nav_msgs::Odometry>("wheel_odom", 50);
	tf::TransformBroadcaster odom_broadcaster;

	// initialize robot position to be {x:0,y:0,th:0}
	double x_pos = 0.0;								//initial X position
	double y_pos = 0.0;								//initial Y position
	double theta = 0.0;								//initial yaw angle
	float wheelSeparation = 0.506;
	float robotLength = 0.43;							//in m
	float OdomTurnMultiplier = 1;         						//might need to scale turns to account for wheel/encoder slippage
	ros::Time NowTime, PrevTime;
	double DL = 0.0;
	double DR = 0.0;
	double ElapsedTime = 0.0;
	ros::Rate rate(10.0);								//publish message at a rate of 10 Hz
	PrevTime = ros::Time::now();

	while (_nh.ok()){

		ros::spinOnce();
		NowTime = ros::Time::now();
		ElapsedTime = (NowTime - PrevTime).toSec();

		DL = (ElapsedTime * gVel_Left);							//compute distance traveled by each wheel
		DR = (ElapsedTime * gVel_Right);

		double delta_xy = 0;

		
			delta_xy = (DL + DR) / 2.0;						//average of distance traveled by each wheel
				
		//double delta_theta = ((DL - DR)/wheelSeparation)*OdomTurnMultiplier;		//determine rotation angle
		double delta_theta = (atan2((DR - DL), wheelSeparation)) * OdomTurnMultiplier;		
		theta += delta_theta;								//compute change in yaw angle

		//constrain theta to range 0 to 2pi		
		if (theta > 2 * M_PI) theta -= 2 * M_PI;
		if (theta < 0) theta += 2 * M_PI;		
		double dx;
		double dy;

		
		dx = delta_xy * cosf(theta);							//compute change in x-position
		dy = delta_xy * sinf(theta);							//compute change in y-position
												
		x_pos += dx;									//new x-position
		y_pos += dy;									//new y-position							

		double vx = delta_xy/ElapsedTime;						//new linear velocity
		double vtheta = delta_theta/ElapsedTime;					//new angular rotation	
		
			
		//initialize odometry transform and odometry message headers
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";

		//publish odometry transform
		odom_quat = tf::createQuaternionMsgFromYaw(theta);				//convert from euler angles to quaternion form

			
	
		odom_trans.header.stamp = NowTime;
		odom_trans.transform.translation.x = x_pos;
		odom_trans.transform.translation.y = y_pos;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//publish estimated position 
		odom.header.stamp = NowTime;
		odom.pose.pose.position.x = x_pos;
		odom.pose.pose.position.y = y_pos;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		//will need to add pose covariance values for use with ekf 

		//publish estimated velocity
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = vtheta; 
		//will need to add twist covariance values for use with ekf 

		odom_broadcaster.sendTransform(odom_trans); 					//publish transform
		odom_pub.publish(odom);								//publish odometry
		PrevTime = NowTime;								//reset loop time for next iteration
		rate.sleep();
	}
}

	
		
		
	
	
