/* 

This is the ROS control node for the rover's wheels, written in c++. This node takes input from the encoders node and 
front end node (joystick), does some PID control (if the motor encoders aren't boken), then sends a command to the 
sabertooths. 

*/

#include <iostream>
#include "ros/ros.h"
#include "wheel_control/wheelSpeed.h"
#include "sensor_msgs/JointState.h"
#include "sabertooth/sabertooth.h"
#include "phidgets_high_speed_encoder/EncoderDecimatedSpeed.h"
#include <std_msgs/Float64.h>
#include "diagnostic_msgs/DiagnosticArray.h"
#include <math.h>
#include <functional>
#include <dynamic_reconfigure/server.h>
#include "dynamic_reconfigure/server.h"
#include "wheel_control/wheel_controlConfig.h"


typedef unsigned char byte;

using namespace std;

/*--------------------Variable Defenitions--------------------*/

// Set points are taken in from the outsde world and used to set target speeds for the control
float setPoints[2] = {0, 0};
/*
 	0 --> Left
	1 --> Right
*/

// We've got different control modes for the wheels
int wheelMode = 1;

const int PIDMode = 0;// --> normal PID control 
const int voltageMode = 1;// --> straight voltage control ie. motor encoders are fucked 

// The range that is written to the sabertooth is 0 to 127 where 0 is full backwards, 64 is stopped, 127 is full forward.
uint16_t motorSpeed[4] = {64, 64, 64, 64};
const int motor_FL = 0; //Front Left
const int motor_FR = 1; //Front Right
const int motor_BL = 2; //Back Left
const int motor_BR = 3; //Back Right

// Read the sabertooth documentation if you want to understand this
uint16_t motorAddress[][2] = {{130, 6},{128, 7},{128, 6},{130, 7}};
/*
	0 --> Front Left
	1 --> Front Right
	2 --> Back Left
	3 --> Back Right
*/

float encoderValues[4];
//encStatus elements are changed from OK to ERROR when the associated encoder breaks
byte OK = diagnostic_msgs::DiagnosticStatus::OK;
byte ERROR = diagnostic_msgs::DiagnosticStatus::ERROR; 
byte encStatus[4] = {OK, OK, OK, OK};  
const int enc_FL = 0; //Front Left
const int enc_FR = 1; //Front Right
const int enc_BR = 2; //Back Right
const int enc_BL = 3; //Back Left

const float stopCommand = 64.0;

// Control gains
float kp = 0.12;
float ki = 0.0012;
float kd = 0.00001;
// Variables used in normalControl(). Declared here so that they don't get redeclared everytime the function runs.

float sp; //motor setpoint
float error;
float encSpeed;
float throttleVal;
int motorVal;
int maxSpeed = 127;
int minSpeed = 0;
// these are arrays so one value is stored for each wheel 
float cumError[4]; 
float lastError[4]; 
float derError[4]; 

float speedMultipliers[4] = {1,-1,1,-1};

/*--------------------Function Defenitions--------------------*/

//These callback functions are called by ros::spin() and ros::spinOnce() in main() and is how we handle incoming messages.
void encStatusCallback(const diagnostic_msgs::DiagnosticArray msg) {
	encStatus[enc_FL] = msg.status[enc_FL].level;
	encStatus[enc_FR] = msg.status[enc_FR].level;
	encStatus[enc_BL] = msg.status[enc_BL].level;
	encStatus[enc_BR] = msg.status[enc_BR].level;
	ROS_INFO("Got Enc Status FL: %d, FR: %d, BL: %d, BR: %d.",encStatus[enc_FL],encStatus[enc_FR],encStatus[enc_BL],encStatus[enc_BR]);
}//end encStatusCallback

void wheelCallback(const wheel_control::wheelSpeed msg) {
	ROS_INFO("I heard: Left: [%f], Right: [%f].\n", msg.left, msg.right);
	setPoints[0] = 2 * msg.left;//*(float)maxSpeed;
	setPoints[1] = 2 * msg.right;//*(float)maxSpeed;
	wheelMode = msg.wheelMode;
}//end wheelCallBack

void encoderCallback(const sensor_msgs::JointState msg) {	
	encoderValues[motor_FL] = msg.velocity[enc_FL];
	encoderValues[motor_FR] = -msg.velocity[enc_FR];
	encoderValues[motor_BL] = msg.velocity[enc_BL];
	encoderValues[motor_BR] = -msg.velocity[enc_BR];

	ROS_INFO("Got encoder values: FL: [%f], FR: [%f].\n", encoderValues[0], encoderValues[1]);
}//end encoderCallBack

void configCallback(wheel_control::wheel_controlConfig &config, uint32_t level){
    speedMultipliers[motor_FL] = config.front_left;
    speedMultipliers[motor_FR] = config.front_right;
    speedMultipliers[motor_BL] = config.back_left;
    speedMultipliers[motor_BR] = config.back_right;

}

// This function tries its darndest to match the setpoint speed on each wheel, its basically a P controller that 
// adjusts a throttle postion after each loop. Basically shitty cruise cotnrol. It kinda comes out to be a PI 
// controller, we dont need no D, we got dirt for that.
void normalControl(int n) {
	// the actual wheel control system that is called from the control loop 
	// loop over wheel motors to get speeds 
		// get setpoint in terms of encoder speed
		sp = 0.0; 
		error = 0; 
		encSpeed = 0; 
		motorVal = 0;
		throttleVal = 0;

		// SP will then be a value from -MaxWheelSpeed to Plus Max Wheel Speed
		if(n == 0)
			sp = setPoints[0]*350;
		else if (n == 1)
			sp = setPoints[1]*350;
		else if (n == 2)
			sp = setPoints[0]*350;
		else if (n == 3)
			sp = setPoints[1]*350;

		// get encoder speed
		encSpeed = encoderValues[n]; // this value should have the same range OR MORE than the SP range 
		// if the range of EncSpeed is less than the range of SP, as when you have a large Max wheel Speed this means the system saturates quickly 
		//cout << "Encoder: "<<n<< " Value is: "<< EncSpeed<< " From Control Loop"<<endl; 
		//Find the difference between the setpoint and where the encoders are 
		error = (sp - encSpeed); 
		cout<<"SetPoint " <<n<<": " << sp<<endl;
		cout<<"Enc " <<n<<": " << encSpeed<<endl;
		cout<<"Error "<<n<<": "<< error<<endl;
		cumError[n] += error; // update the cum error 
		derError[n] = (error - lastError[n]); // update the der error 
		if(cumError[n]*error <0){
			/// this means we had a zero crossing and we should zero the CumError and the Last Error 
			// a zero crossing is the sign of the error changed 
			cumError[n] = 0.0;
			derError[n] = 0.0;
			// in proper this does not need to be done, but it was added to try to improve settling time, normally you would have to have a large 
			/// overshoot to kill the cum error 
		}
		// set a throttle value based on the gains and the errors 
		// everything should be kept as a flot going in, and then when we modify the motor vals it becomes and int 
		throttleVal = kp * error + ki * cumError[n] + kd * derError[n]; 
		//apply the throttle change to the Motor val
		// the throttle value is used to move the motor speed (voltage applied) from the pervious loop into the present 
		// MOTOR VAL HAS TO BE AN INT. The motor controllers can only take in ints as a motor speed value 
		if(n==1 || n==3 ){
			// these are the right motors, they're backwards becasues reasons 
			motorVal = motorSpeed[n] - round(throttleVal);
		}else{
			motorVal = motorSpeed[n] + round(throttleVal);
		}
		// bound the motor speed
		// the purpose of this was to not give the motor controller a value outside of the expected range. This acts as a forced saturation on the motor controller in code 
		// the motor controller should also do this itself, but this was done by the Departmant of Reduandacy Department to be redundant. 
		if(motorVal>maxSpeed){
			motorVal = maxSpeed;
		}else if(motorVal<minSpeed){
			motorVal = minSpeed;
		}
		//if(setPoint == 0) motorVal = 64;
		// re assign the motor speed
		cout<<"motorVal " <<n<<": " << motorVal<<endl;
		motorSpeed[n] = motorVal; // after its been bounded we write this to the valeu taht we are going to write
//		cout<<"Writing Motor Send: "<<MotorVal<<" EncoderValue: "<<EncSpeed<<" Index: "<< n<<" Error: "<< Error << " ThrottleVal:"<<ThrottleVal << " SetPoint:"<< SP <<endl;
		lastError[n] = error;
	// loop over wheels to write
	// these were done seperatly to redunce the delay in writing to all the motors incase of large change so we dont have weird jerks 
}//end normalControl

//This is used if all the encoders are fucked
void voltageControl() {
	motorSpeed[motor_FL] = setPoints[0]*63.0*speedMultipliers[motor_FL] + stopCommand;
	motorSpeed[motor_FR] = -setPoints[1]*63.0*speedMultipliers[motor_FR] + stopCommand;
	motorSpeed[motor_BL] = setPoints[0]*63.0*speedMultipliers[motor_BL] + stopCommand;
	motorSpeed[motor_BR] = -setPoints[1]*63.0*speedMultipliers[motor_BR] + stopCommand;
}//end voltageControl

void eStop() {
	for(int i; i<4; i++)
		motorSpeed[i] = stopCommand; 
}//end eStop

void EncFailureHandler(){ 
	if((encStatus[enc_FL]==ERROR && encStatus[enc_BL]==ERROR) || (encStatus[enc_FR]==ERROR && encStatus[enc_BR]==ERROR))
		voltageControl();  
	for(int i=0; i<4; i++){
		if(encStatus[i] == diagnostic_msgs::DiagnosticStatus::ERROR){
			if(i==0)
				motorSpeed[i] = motorSpeed[2];
			else if(i==1)
				motorSpeed[i] = motorSpeed[3];
			else if(i==2)
				motorSpeed[i] = motorSpeed[0];
			else if(i==3)
				motorSpeed[i] = motorSpeed[1];	
		}	
	}	
}//end EncFailureHandler

int main(int argc, char **argv) {

	//set up the ROS node
	ros::init(argc, argv, "wheelControl");
	ros::NodeHandle n;
	ros::Subscriber joystickSub = n.subscribe("wheelSpeedTopic", 4, wheelCallback);
	ros::Publisher pub = n.advertise<sabertooth::sabertooth>("sabertooth", 1);

	ros::Publisher setpointLeft = n.advertise<std_msgs::Float64>("setpointLeft", 100);
	ros::Publisher setpointRight = n.advertise<std_msgs::Float64>("setpointRight", 100);

	ros::Subscriber enc = n.subscribe("joint_states", 1, encoderCallback);
	ros::Subscriber encState = n.subscribe("encoder_status", 1, encStatusCallback);

	dynamic_reconfigure::Server<wheel_control::wheel_controlConfig> server;
    dynamic_reconfigure::Server<wheel_control::wheel_controlConfig>::CallbackType f;
    f = boost::bind(configCallback, _1, _2);
    server.setCallback(f);


    ros::Rate loop_rate(80);
	ros::Rate send_rate(20);

	cout << "Control node ready." << endl;
	cout<< "This built to this"<<endl;

	//the whole node loops on this while loop
	while (ros::ok()) {
	  wheelMode = voltageMode; //remove this if you wnat PID mode --------------------------------------
	  for (int i = 0; i < 4; i++) {
            if (wheelMode == PIDMode)
              normalControl(i);
            else if (wheelMode == voltageMode)
              voltageControl();
            else
              eStop();

	  //EncFailureHandler();

            	//sending the sabertooth message
            	sabertooth::sabertooth msg;

            	msg.data = motorSpeed[i];
            	msg.address = motorAddress[i][0];
            	msg.command = motorAddress[i][1];
            	ROS_INFO("Sending Speed [%d] to motor [%d] with multiplier [%f].\n", motorSpeed[i], i, speedMultipliers[i]);
            	pub.publish(msg);
	    ros::spinOnce();
            send_rate.sleep();
	  }
	loop_rate.sleep();
	}

	return 0;

}//end main
