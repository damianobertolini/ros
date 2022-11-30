#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
//#include <eigen3/Eigen/Eigen>
#include <sstream>

#include "robot.cpp"


using namespace std;


//ros::Subscriber sub;
//sensor_msgs::JointState js;

/*
void msg_callback(const std_msgs::String::ConstPtr& msg){
    cout << "in msg_callback" << endl;
    //ROS_INFO("I heard: [%f]", js->velocity.c_str());
    ROS_INFO("msg");
	ROS_INFO("I heard: [%s]", msg->data.c_str());
    cout << "msg: " << msg->data.c_str() << endl;
    cout << "---------------------------------------------";
	//js=joint;
}

void js_callback(const sensor_msgs::JointState& joint){

    //ROS_INFO("I heard: [%f]", js->velocity.c_str());
    ROS_INFO("js");
	
    cout << "vel: " << joint.position[0]<< endl;
    cout << "vel: " << joint.position[1]<< endl;
    cout << "vel: " << joint.position[2]<< endl;
    cout << "---------------------------------------------";
	//js=joint;
}
*/

void start_chatter(int argc, char ** argv){
    ros::init(argc, argv, "main_cpp");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Publisher chatter_pub = n.advertise < std_msgs::String > ("chatter", 1000);
	

    int count = 0;
    
    while (ros::ok()) {
        
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
        //ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        //ros::spinOnce();
        loop_rate.sleep();
        ++count;
        cout << "count: " << count << endl;
		
		//sensor_msgs::JointState js = ros::topic::wait_for_message("/ur5/joint_stats", sensor_msgs::JointState);
		//cout << "pos1: " << js.position[0]<< endl;
        
       loop_rate.sleep();
    }
}


void call_robot(int argc, char ** argv){
/*
    cout << "in robot" << endl;
    ros::init(argc, argv, "robot_cpp");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    //ros::AsyncSpinner spinner(0);
    //spinner.start();
    //sub = n.subscribe("/ur5/joint_states", 1, js_callback);

    ros::Subscriber sub = n.subscribe("chatter", 10, robot_msg_callback);

    ros::spin();
    
    cout << "done spinning " << endl;
    */

    Robot r;
    r.start_callback(argc, argv);
}

int main(int argc, char ** argv) {
	
    thread robot_thread(call_robot, argc,argv); //fa partire la classe robot in un thread così può fare spin all'infinito
    thread chatter_thread(start_chatter, argc,argv);
    cout << "inizio" << endl;

    //Robot robot(argc,argv);
	/*
    ros::init(argc, argv, "test_cpp");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    //sub = n.subscribe("/ur5/joint_states", 1, js_callback);
    ros::Subscriber sub = n.subscribe("chatter", 10, msg_callback);
    */
    Robot r;
    int i=0;
    while(ros::ok){
        cout << "waiting: " << i << endl;
        sleep(1);
        cout << "joints: " << Robot::joint.position << endl;
    }

    cout << "fine" << endl;
    
    return 0;
}
