
#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
//#include <eigen3/Eigen/Eigen>
#include <sstream>
#include "Helper.cpp"
#include "Kin.cpp"
#include "robot.cpp"

#include <iostream>
#include <fstream>





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
    ros::Rate loop_rate(5);
    ros::Publisher chatter_pub = n.advertise < std_msgs::String > ("chatter", 10);
	

    int count = 0;

    //Kin kin;
    //kin.test_pinocchio();
    //kin.test_pinocchio2();
    //kin.test_pinocchio3();

    
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
        //cout << "count: " << count << endl;
		
		//sensor_msgs::JointState js = ros::topic::wait_for_message("/ur5/joint_stats", sensor_msgs::JointState);
		//cout << "pos1: " << js.position[0]<< endl;
        
       //loop_rate.sleep();
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

    Robot r(true,argc,argv);
    r.start_callback(argc, argv);
}

int main(int argc, char ** argv) {
	Helper h;
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


    cout << "partito" << endl;

    cout << "testing kin" << endl;

    Kin kin;
    Eigen::Vector < double, 6 > q = Eigen::Vector < double, 6 > (0.031010589797616728,-0.6633989402088383,-0.32539011740993384,-0.029282072634296497,0.025924898063880653,6.225361921304502e-17);
    kin.compute_fc(q);
    //ee pos :      0.680509    0.131313   0.642843
    Eigen::Vector3d pos = kin.get_ee_p();

    cout << "starting position" << q << endl;

    cout << "end effector pos =\n " << pos;

    cout << "------------------------------------------------"<< endl;

    //kin.compute_J(q);

    Eigen::Vector3d pos2(-0.8, -0.2, 0.8);
    Eigen::Vector3d rpy(0,0,M_PI);

    vector<Eigen::Vector < double, 6 >> path = kin.compute_ik(pos2,rpy, q, 50, false);


    

    ofstream myfile;
    myfile.open ("path2.txt");
    for (Eigen::Vector < double, 6 > i: path)
            myfile << i << "\n\n";
    myfile.close();


    kin.compute_fc(path[path.size()-1]);
    //ee pos :      0.680509    0.131313   0.642843
    Eigen::Vector3d pos3 = kin.get_ee_p();

    cout << "end q position" << q << endl;

    cout << "end effector pos =\n " << pos3;

    cout << "------------------------------------------------"<< endl;



    bool start=false;
    Robot robot(false, argc,argv);
    sensor_msgs::JointState joint;

    while(!Robot::started){
        sleep(0.51);//aspetto di avere un feedback dal robot
    }
    cout << "robot, ready\n ";
    cout << "robot pos: \n";
    Robot::print_position(Robot::joint);


    string s;
    cout << "cin to stall";
    cin >> s;




    //joint.position = {0.031010589797616728,-0.6633989402088383,-0.32539011740993384,-0.029282072634296497,0.025924898063880653,6.225361921304502e-17};
    //robot.publish(joint, argc, argv);
    sleep(0.51);
    /*
    kin.compute_fc({0.031010589797616728,-0.6633989402088383,-0.32539011740993384,-0.029282072634296497,0.025924898063880653,6.225361921304502e-17});
    cout << "initial  pos: " << kin.get_ee_p();
    kin.compute_fc(path[path.size()]);
    cout << "final pos: " << kin.get_ee_p();
    //Eigen::Map<Eigen::Vector>(joint.position) = path[i];

    //mando posizioni
    //--------------------------------------------------------------------------------

    ros::Rate loop_rate(1000);

    for(int i=0;i < path.size();i++){
       
        joint.position = {path[i].coeff(0), path[i].coeff(1), path[i].coeff(2), path[i].coeff(3), path[i].coeff(4), path[i].coeff(5)};
        //cout << "posizioni passate " << endl;
        
        robot.publish(joint, argc, argv);
        //sleep(0.001);
        loop_rate.sleep();
    }

    pos2={0.8, 1.5, 0.7};
    rpy={-M_PI,0,M_PI};
    path = kin.compute_ik(pos2,rpy, q, 10000, false);

    for(int i=0;i < path.size();i++){
        
        joint.position = {path[i].coeff(0), path[i].coeff(1), path[i].coeff(2), path[i].coeff(3), path[i].coeff(4), path[i].coeff(5)};
        
        //cout << "posizioni passate " << endl;
        
        robot.publish(joint, argc, argv);
        //sleep(0.001);
        loop_rate.sleep();
    }

    kin.compute_fc(path[path.size()]);

    cout << "final pos: " << kin.get_ee_p();

    kin.test_tot2eul();

    cout << "fine test kin" << endl;
    //------------------------------------------------------------------------------
    */
/*
    while(true){
        
        sleep(5);
        Robot::print_position(Robot::joint);
        //joint.position={0.0,0.0,0.0,0.0,0.0,0.0};
        //robot.publish(joint, argc, argv);
        sleep(1);
        //Robot::print_position(Robot::joint);
        //sleep(5);


        


        //joint.position={3,2.1,0.5,-2.7,2,-1.0};
        //robot.publish(joint, argc, argv);
        //sleep(1);
        //Robot::print_position(Robot::joint);
        
    }

    */
    int i=0;
    cout << "waiting: " << i << endl;
    while(ros::ok){
        
        sleep(1);
        cout << "joints: ";
        Robot::print_position(Robot::joint);
    }
    joint=Robot::joint;
    //robot.publish(joint, argc, argv);

    cout << "fine" << endl;
    
    return 0;
}

