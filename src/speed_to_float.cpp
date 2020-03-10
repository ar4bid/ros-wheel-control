#include <iostream>
#include "ros/ros.h"
#include "phidgets_high_speed_encoder/EncoderDecimatedSpeed.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

ros::Publisher *pub;

void callback(const phidgets_high_speed_encoder::EncoderDecimatedSpeed msg){
	std_msgs::Float64 speed;

	speed.data = msg.avr_speed;
	pub->publish(speed);

}

void callback_instant(const sensor_msgs::JointState msg){
	float val = msg.velocity[3];
	std_msgs::Float64 speed;

	speed.data = val;
	pub->publish(speed);

}

int main(int argc, char** argv){
	ros::init(argc, argv, "speed-to-float");
	ros::NodeHandle node;
	pub = new ros::Publisher(node.advertise<std_msgs::Float64>("output", 1));
	ros::Subscriber sub = node.subscribe("input", 1, callback);
	//ros::Subscriber sub = node.subscribe("input", 1, callback_instant);

	ros::spin();
	
	delete pub;
}
