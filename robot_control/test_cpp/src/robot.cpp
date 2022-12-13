#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>

#include <sstream>
#include <iostream>
#include <string>
#include <thread>

using namespace std;


void robot_msg_callback(const std_msgs::String::ConstPtr& message);


void js_callback(const sensor_msgs::JointState& joint);




class Robot{
    public: 

        static inline sensor_msgs::JointState joint;
        static inline std_msgs::String msg;
        static inline int stat=0;
        static inline bool started = false;
    
        Robot(){
            //vuoto
        };

        Robot(bool init){//da chiamare una volta per non avere errori
            if(init){
                joint.position={0.0,0.0,0.0,0.0,0.0,0.0};
            }
        };

        Robot(int argc, char ** argv){
            cout << "in robot" << endl;

            std::stringstream ss;
            ss << "hello world ";
            msg.data = ss.str();
            //std_msgs::String msg = you_initial_value_here;
            /*
            ros::init(argc, argv, "robot_cpp");
            ros::NodeHandle n;
            ros::Rate loop_rate(1);

            //ros::AsyncSpinner spinner(0);
            //spinner.start();


            //sub = n.subscribe("/ur5/joint_states", 1, js_callback);
            ros::Subscriber sub = n.subscribe("chatter", 10, robot_msg_callback);
            */
            cout << "done constructor" << endl;
        };

        void spin(){
            ros::spinOnce();
        }

        void start_callback(int argc, char ** argv){
            cout << "in robot" << endl;
            ros::init(argc, argv, "robot_cpp");
            ros::NodeHandle n;
            ros::Rate loop_rate(1);

            //ros::AsyncSpinner spinner(0);
            //spinner.start();
            //sub = n.subscribe("/ur5/joint_states", 1, js_callback);
            //ros::Subscriber sub = n.subscribe("chatter", 10, robot_msg_callback);
            ros::Subscriber sub_js = n.subscribe("/ur5/joint_states", 1, js_callback);

            cout << "start spinning " << endl;

            ros::spin();
            
            cout << "done spinning " << endl;
        }

        //sensor_msgs::JointState& get_latest_js(){
        //    return 0;
        //}

        void test(){
            cout << "robot tested" << endl;
        }
        void set_msg(string s){
            //msg.data = s;
        }
        string get_msg(){
            return msg.data;
        }
        static int print_position(sensor_msgs::JointState j){
            
            //Robot r;
            //cout << j << endl;

            if(!Robot::started){
                cout << "nessun messaggio joint" << endl;
                return -1;
            }

            //cout << "printing joints" << endl;

            for(int i=0;    i< 6;i++){
                cout << "posizione " << i << ": " << j.position[i] << endl;
            }
            //cout << "done printing" << endl;
            cout << "--------------------------------------------------" << endl << endl;
            return 1;
        };
};

//std_msgs::String Robot::msg = new std_msgs::String();


void robot_msg_callback(const std_msgs::String::ConstPtr& message){
    cout << "in msg_callback" << endl;
    //ROS_INFO("I heard: [%f]", js->velocity.c_str());
    //ROS_INFO("msg");
	ROS_INFO("I heard: [%s]", message->data.c_str());
    //cout << "msg: " << message->data.c_str() << endl;
    cout << "---------------------------------------------" << endl;
    Robot::stat++;
    cout << "stat: " << Robot::stat << endl;
	//js=joint;

    //Robot robot;
    //robot.test();
    //robot.set_msg(message->data.c_str());
    //robot.msg.data = message->data.c_str();

    //std_msgs::String m;
    //m.data = message->data.c_str();
    //std_msgs::String Robot::msg = m;
}

void js_callback(const sensor_msgs::JointState& j){

    //ROS_INFO("I heard: [%f]", js->velocity.c_str());
    //ROS_INFO("js");
	/*
    cout << "vel0: " << j.position[0]<< endl;
    cout << "vel1: " << j.position[1]<< endl;
    cout << "vel2: " << j.position[2]<< endl;
    cout << "vel3: " << j.position[3]<< endl;
    cout << "vel4: " << j.position[4]<< endl;
    cout << "vel5: " << j.position[5]<< endl;
    cout << "---------------------------------------------";
	//js=joint;
    */
    Robot::joint = j;
    Robot::started = true;
}
