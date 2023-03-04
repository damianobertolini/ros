
#include "ros/ros.h"

//#include "test_cpp/Pointxyz_cpp.h"
//#include "test_cpp/Block_cpp.h"
//#include "test_cpp/Blocklist_cpp.h"

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

//#include "msg/Pointxyz.h"
//#include "msg/Block.h"
//#include "msg/BlockList.h"
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
//#include <eigen3/Eigen/Eigen>
#include <sstream>
#include "Helper.cpp"
#include "Kin.cpp"
#include "robot.cpp"
//#include "../msg/Block.msg"
//#include "../msg/Pointxyz.msg"
//#include "../msg/BlockList.msg"

//#include "object_msgs/Object.h"
#include "my_vision_messages/Pointxyz.h"
#include "my_vision_messages/Block.h"
#include "my_vision_messages/BlockList.h"
//#include "/home/massiccio/ros_ws/devel/include/messages/Pointxyz.h"
//#include "/home/massiccio/ros_ws/devel/include/object_msgs/Object.h"
//#include "/home/massiccio/ros_ws/devel/include/messages//Block.h"
//#include "/home/massiccio/ros_ws/devel/include/messages//BlockList.h"



#include <iostream>
#include <fstream>

#define STEPS 30
#define SAFE_Z 0.65



using namespace std;



void start_chatter(int argc, char ** argv){
    ros::init(argc, argv, "main_cpp");
    ros::NodeHandle n;
    ros::Rate loop_rate(5);
    ros::Publisher chatter_pub = n.advertise < std_msgs::String > ("chatter", 10);

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
        //cout << "count: " << count << endl;
		
		//sensor_msgs::JointState js = ros::topic::wait_for_message("/ur5/joint_stats", sensor_msgs::JointState);
		//cout << "pos1: " << js.position[0]<< endl;
        
       //loop_rate.sleep();
    }
}

void block_listener_callback(const my_vision_messages::Block::ConstPtr & b){

    ROS_INFO("we are in vision callback");
    //ROS_INFO("js");
	/*
    cout << "class_number: " << b->class_number<< endl;
    cout << "point: " << b->point<< endl;
    cout << "rot angle: " << b->rot_angle<< endl;
    cout << "top_btm_l_r[0]: " << b->top_btm_l_r[0]<< endl;
    cout << "up_dw_stand_lean: " << b->up_dw_stand_lean<< endl;
    cout << "confidence: " << b->confidence<< endl;
    cout << "---------------------------------------------";
	//js=joint;
    */
    int i=0;
    //print_position();
}


void start_block_callback(int argc, char ** argv){
    cout << "in block" << endl;
    ros::init(argc, argv, "main_cpp_block_listener");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    //ros::AsyncSpinner spinner(0);
    //spinner.start();
    //sub = n.subscribe("/ur5/joint_states", 1, js_callback);
    //ros::Subscriber sub = n.subscribe("chatter", 10, robot_msg_callback);
    ros::Subscriber sub_js = n.subscribe("/ur5/vision_results", 1, block_listener_callback);

    cout << "start spinning blocks" << endl;
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    cout << "done spinning blocks" << endl;
}


void call_robot(int argc, char ** argv){

    Robot r(true,argc,argv);
    r.start_callback(argc, argv);
}

int main(int argc, char ** argv) {
	Helper help;
    Kin kin;

    //object_recognition_msgs::ObjectInformation oi;

    
    thread robot_thread(call_robot, argc,argv); //fa partire la classe robot in un thread così può fare spin all'infinito
    //thread chatter_thread(start_chatter, argc,argv);
    thread block_listener_thread(start_block_callback, argc,argv);

    Robot robot(false, argc,argv);
    sensor_msgs::JointState joint;
    float delta;
    Eigen::Vector3d p_theo;
    Eigen::Vector3d p_real;

    while(!Robot::started){
        sleep(0.51);//aspetto di avere un feedback dal robot
    }
    cout << "robot, ready\n ";
    cout << "robot pos: \n";
    Robot::print_position(Robot::joint);
    Eigen::Vector < double, 6 > q;
    Eigen::Vector < double, 6 > pr_i;
    Eigen::Vector < double, 6 > pr_f;
    string s;
    float f;

/*


    help.fill_pr_i_f(pr_i,pr_f);

    cout << "\n------------------------------------------------------------\n" ;

    q << 0.1518,   -0.1912,    0.4505 ,0,0,0;
    
    
    robot.move_to(pr_i,3000,1,false);//sposto alla posizione iniziale voluta

    cout << "fatto" << endl;
    q=robot.j_to_q(Robot::joint);//dove sono adesso

    kin.compute_fc(q);//finale teorica
    
    p_theo << pr_i[0],pr_i[1],pr_i[2];
    q = robot.j_to_q(Robot::joint);//prendo la posizione reale
    kin.compute_fc(q);
    p_real= kin.get_ee_p();

    delta = help.dist(p_theo,p_real);

    cout << "teorico: " << p_theo << endl;
    cout << "reale: " << p_real << endl;
    cout << "delta: " << delta << endl;

    //Robot::print_position();

    cout << "\n------------------------------------------------------------\n";
    sleep(3);

    while(true){
        //0.5 0.2 0.7
        pr_f = Eigen::Vector<double, 6> ((rand()%10)*0.1-0.5,(rand()%10)*0.1-0.5, (rand()%2)*0.1+0.6,
            0,0,0);
        //help.fill_pr_i_f(pr_i,pr_f);
        pr_f = Eigen::Vector<double,6>();
        
        cout << "cin per stalling... 0 per muove il braccio, altrimenti un valore per il gripper";
        cin >> f;//f float

        //robot.set_gripper(f);

        if(f!=0){
            robot.publish_grip(f);
            //cout << "\nset gripper to: " << f << endl;
        }else{
            help.fill_pr_next(pr_f);

        //cout << "moving to" << pr_f << endl;

            robot.move_to(pr_f,3000,f,false);
            kin.compute_fc(pr_f);//finale teorica
        
            p_theo << pr_f[0],pr_f[1],pr_f[2];
            q = robot.j_to_q(Robot::joint);//prendo la posizione reale
            kin.compute_fc(q);
            p_real= kin.get_ee_p();

            delta = help.dist(p_theo,p_real);

            cout << "teorico: " << p_theo << endl;
            cout << "reale: " << p_real << endl;
            cout << "delta: " << delta << endl;
        }

*/

        //Robot::print_position();

        sleep(3);

        //robot.set_gripper(0);

        cout << "\n------------------------------------------------------------\n";
        int kiuy = 0;
        ros::Rate loop_rate(1);
        while(ros::ok){
            sleep(0.5);
            cout << kiuy << endl;
            kiuy++;
            kiuy = kiuy%20;
            loop_rate.sleep();
        }

        cout << "fine";
        
    
/*
    cout << "inizio" << endl;
    cout << "testing kin" << endl;

    Eigen::Vector < double, 6 > q = Eigen::Vector < double, 6 > (0.183735039827282,	-1.14288069226790,	1.00754157494767,	-1.43545720947467,	1.57079632679490,	1.38706128696761);
    //ee pos :     [-0.629136; -0.252495;  0.701697];
    //-0.629135719285118 -0.252494671044228 0.701696928465377
    cout << "------------------------------------------------"<< endl;

    //kin.compute_J(q);
    Eigen::Vector3d pos2(-0.6, -0.2, 0.6);
    Eigen::Vector3d rpy(-M_PI/6, -M_PI/3, -M_PI/4);
    Eigen::Vector < double, 6 > pr_i;
    Eigen::Vector < double, 6 > pr_f;

    help.fill_pr_i_f(pr_i,pr_f);

    kin.compute_fc(q);
    pr_i(0) = kin.get_ee_p()[0];
    pr_i(1) = kin.get_ee_p()[1];
    pr_i(2) = kin.get_ee_p()[2];
    pr_i(3) = kin.rotm2eul(kin.T0e)[0];
    pr_i(4) = kin.rotm2eul(kin.T0e)[1];
    pr_i(5) = kin.rotm2eul(kin.T0e)[2];

    cout << "iniziale: " << pr_i << endl << endl;
    cout << "finale: " << pr_f << endl << endl;

    Eigen::MatrixXd k = help.fillK(0.1);

    cout << "\nfilled k\n";

    vector<Eigen::Vector < double, 6 >> path_theory = help.fillpath(q,pr_f, STEPS);

    cout << "\nfilled theo path\n";

    //vector<Eigen::Vector < double, 6 >> path = kin.compute_ik(pos2,rpy, q, 50, false);

    vector<Eigen::Vector < double, 6 >> path = kin.da_a(path_theory,q,k,STEPS);

    cout << "\nfilled da a path\n";

    ofstream myfile;

    myfile.open ("path2.txt");
    for (Eigen::Vector < double, 6 > i: path)
            myfile << i(0)<< "," <<i(1)<< "," <<i(2)<< "," <<i(3)<< "," <<i(4)<< "," <<i(5)<< "\n";
    myfile.close();

    cout << "\nprinted path to file\n";

    kin.compute_fc(path[path.size()-1]);
    
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
    cout << "cin to stall ";
    cin >> s;

    sleep(0.51);

    int i=0;
    cout << "waiting: " << i << endl;

    int path_i=0;
    int path_max = path.size();
    ros::Rate loop_rate(5);

    while(ros::ok){
        
        sleep(0.5);
        //cout << path_i << ")\n";
        //Robot::print_position(Robot::joint);
        robot.publish(path[path_i]);
        //ros::sleep(0.2);
        
        path_i++;
        path_i = path_i%path_max;

        loop_rate.sleep();
    }
    joint=Robot::joint;
    //robot.publish(joint, argc, argv);

    cout << "fine" << endl;
    */
    return 0;
}

