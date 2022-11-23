#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>

#include <sstream>


using namespace std;


ros::Subscriber sub;
sensor_msgs::JointState js;


void js_callback(const sensor_msgs::JointState& joint){

    //ROS_INFO("I heard: [%f]", js->velocity.c_str());
    ROS_INFO("js");
	
    cout << "vel: " << joint.position[0]<< endl;
    cout << "vel: " << joint.position[1]<< endl;
    cout << "vel: " << joint.position[2]<< endl;
    cout << "---------------------------------------------";
	js=joint;
	sub.shutdown();
}

void get_last_js(){
	ros::NodeHandle n;
    sub = n.subscribe("/ur5/joint_states", 1, js_callback);
	
	
}
int main(int argc, char ** argv) {
	

    cout << "inizio" << endl;
	
    ros::init(argc, argv, "test_cpp");
    loop_rate.sleep();

    get_last_js();
	    ros::Rate loop_rate(1);

	cout << "pos1: " << js.position[0]<< endl;
    cout << "pos2: " << js.position[1]<< endl;
    cout << "pos3: " << js.position[2]<< endl;
	
	/*
	
        ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise < std_msgs::String > ("chatter", 1000);
	

    int count = 0;
    
    while (ros::ok()) {
        
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
		
		//sensor_msgs::JointState js = ros::topic::wait_for_message("/ur5/joint_stats", sensor_msgs::JointState);
		//cout << "pos1: " << js.position[0]<< endl;
        
       loop_rate.sleep();
    }
	*/
    cout << "fine" << endl;
    
    return 0;
}
