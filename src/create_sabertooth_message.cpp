#include "ros/ros.h"
#include "sabertooth/sabertooth.h"
#include "std_msgs/Float64.h"

ros::Publisher * pubPtr;
int maxSpeed = 350;
int minSpeed = -maxSpeed;
float speedRange = maxSpeed - minSpeed;
int address = 130;
int command = 7;
float dataRange = 127.0;

void callback(const std_msgs::Float64 msg){
	sabertooth::sabertooth sabertoothMessage;
	sabertoothMessage.address = address;
	sabertoothMessage.command = command;
	
	float val = ( ((msg.data) - (float)minSpeed) * dataRange) / speedRange;
	sabertoothMessage.data = val; //TODO:: remove this
	ROS_INFO_STREAM("Generating Message from "<< msg.data << " became: " << sabertoothMessage.data);
	pubPtr->publish(sabertoothMessage);

}
int main(int argc, char** argv){
	ros::init(argc, argv, "sabertoothMessageGenerator");
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("set_sabertooth", 10, callback);
	pubPtr = new ros::Publisher(node.advertise<sabertooth::sabertooth>("output", 10));	

	node.getParam("max_speed", maxSpeed);
	node.getParam("min_speed", minSpeed);
	node.getParam("address", address);
	node.getParam("command", command);
	
	speedRange = maxSpeed - minSpeed;	
	ROS_INFO_STREAM("Sabertooth Command Generator Ready. Max: "<<maxSpeed<<" Min: "<<minSpeed);
	ros::spin();
	return 0;
}
