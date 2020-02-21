#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>

volatile float x_rel_pos = 0;

void partCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
	x_rel_pos = msg->data[0] - 0.5;
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "follow_me_node");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/Zordon/cmd_vel", 10);
	ros::Subscriber sub = nh.subscribe("/part", 10, &partCallback);

	ros::Rate rate(3);

	while(ros::ok()) {
		geometry_msgs::Twist msg;
		
		if(fabs(x_rel_pos) < 0.1) {
			msg.angular.z = 0.0;
			msg.linear.x = 0.0;
		} else if (x_rel_pos < 0) {
			msg.angular.z = -1.8*x_rel_pos;
			msg.linear.x = 0.08;
		} else if(x_rel_pos > 0) {
			msg.angular.z = -1.8*x_rel_pos;
			msg.linear.x = 0.08;
		}

		pub.publish(msg);

		rate.sleep();
		ros::spinOnce();
	
	}
}
