#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
//#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>

#include <sstream>
#include <iostream>
#include <string>
#include <thread>
#include "Helper.cpp"



using namespace std;


void robot_msg_callback(const std_msgs::String::ConstPtr& message);


void js_callback(const sensor_msgs::JointState& joint);




class Vision{
    public: 

        static inline my_vision_messages::BlockList block_list;
        static inline bool started = false;

        Vision(){
            //vuoto
        };

        void start_callback(int argc, char ** argv){
            cout << "in vision" << endl;
            ros::init(argc, argv, "vision_cpp_callback");
            ros::NodeHandle n;
            ros::Rate loop_rate(1);

            //ros::AsyncSpinner spinner(0);
            //spinner.start();
            //sub = n.subscribe("/ur5/joint_states", 1, js_callback);
            //ros::Subscriber sub = n.subscribe("chatter", 10, robot_msg_callback);
            ros::Subscriber sub_js = n.subscribe("/vision_results", 1, vision_callback);

            cout << "start spinning vision" << endl;
            ros::MultiThreadedSpinner spinner(2);
            spinner.spin();
            cout << "done spinning vision" << endl;
        }

    
        void print_f64j(std_msgs::Float64MultiArray f64j){
            for(int i=0; i< f64j.data.size();i++){
                cout << "\nf64j.data[" << i << "] " << f64j.data[i];
            }
            cout << "\n\n";
        }


        void test(){
            cout << "vision tested" << endl;
        }
        static int print_position(){
            return print_position(joint);
        };

};

//std_msgs::String Robot::msg = new std_msgs::String();




void vision_callback(const my_vision_messages::BlockList& block_list_callb){
    Vision::started = true;
    cout << "in vision callback\n";
}



void swap(double& f1,double& f2){
    float buff;
    buff=f1;
    f1=f2;
    f2=buff;
}
