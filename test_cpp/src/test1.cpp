#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>

#include <sstream>


void js_callback(const sensor_msgs::JointState& js){
  ROS_INFO("I heard: [%s]", js->velocity.c_str());
    cout << "joint trovato"
}


int main(int argc, char ** argv) {
	
	
    ros::init(argc, argv, "test_cpp");
    ros::NodeHandle n;
    //ros::Publisher chatter_pub = n.advertise < std_msgs::String > ("chatter", 1000);
    ros::Subscriber sub = n.subscribe("/joint_states", 1000, js_callback);
    ros::Rate loop_rate(10);
    
    int count = 0;
    /*
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
    }
    */
    return 0;
}
