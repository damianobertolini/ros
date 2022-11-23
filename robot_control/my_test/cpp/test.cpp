

// #include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <iomanip>
//#include <robot.hpp>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
# include "sensor_msgs/JointState.h"
//#include "gazebo_ros_link_attacher/Attach.h"
//#include "std_msgs/Float64.h"
//#include "callbacks.hpp"
#include <eigen3/Eigen/Eigen>
//#include "../include/ur5.hpp" //cambiato a caso [tolto ""]
#include <cmath>
#include <string>
#include <sstream>
//#include "BrickType.hpp"
//#include "Brick.hpp"
//#include "ros_utils.hpp"


using namespace std;
//using namespace ros;

void joint_callback(const sensor_msgs::JointState& msg){
	cout << msg;
}

	
int main(int argc, char **argv){
	
	//------------------------
	ros::init(argc, argv, "hello_world_node");
	// Start the node resource managers (communication, time, etc)
	ros::start();
	// Broadcast a simple log message
	ROS_INFO_STREAM("Hello, world!");
	// Process ROS callbacks until receiving a SIGINT (ctrl-c)
	ros::spin();
	// Stop the node's resources
	ros::shutdown();
		
	//----------------------------
	
	
	string test = "testing";
	
	ros::init(argc,argv,"cpp_node");
	
	ros::NodeHandle n;
	
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("cpp_topic", 1000);
	
	ros::Rate loop_rate(10);
	
	
	//ros::Subscriber sub = n.subscribe("/ur5/joint_states", 1, joint_callback);
	//sensor_msgs::JointState js =  ros::topic::waitForMessage("/ur5/joint_states");
	//cout << js;
	
	int count=0;
	
	while(ros::ok){
		
		
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		
		
		ROS_INFO("%s", msg.data.c_str());
		
		
		chatter_pub.publish(msg);
			
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}
	
	
	cout << "test";
	
	
	return 0;
}

