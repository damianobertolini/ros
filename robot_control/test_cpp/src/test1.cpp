
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
#include "Eigen/Eigen/Dense"
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

/** @file main.cpp
 *  @brief main function for robot procedures
 *
 *  @author Massimo Girardelli
 *  @author massimo Girardelli 2
 *  @bug No know bugs.
 */


using namespace std;

Helper helper;
my_vision_messages::BlockList BlockList_msg;
int block_call_index=0;//per tenere traccia delle call

int procedure(Robot,my_vision_messages::BlockList);

int testing(Robot);

int feed(Robot);

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

void block_listener_callback(const my_vision_messages::BlockList & b){

    //ROS_INFO("we are in vision callback, size: [%d]",b.blocks.size());

    BlockList_msg = b;
    block_call_index++;
    /*
    for(int i=0; i<b.blocks.size();i++){

        Eigen::Vector3d xyz_cam;
        
        xyz_cam << b.blocks[i].point.x ,b.blocks[i].point.y, b.blocks[i].point.z;
        cout << "block: " <<i<< endl<<endl;
        cout << "class_number: " << b.blocks[i].class_number<< endl;
        cout << "point: " << b.blocks[i].point<< endl;
        cout << "rot angle: " << b.blocks[i].rot_angle<< endl;
        cout << "top_btm_l_r[0]: " << b.blocks[i].top_btm_l_r[0]<< endl;
        cout << "up_dw_stand_lean: " << b.blocks[i].up_dw_stand_lean<< endl;
        cout << "confidence: " << b.blocks[i].confidence<< endl;

        Eigen::Vector3d xyz_world = helper.cam_to_world(xyz_cam);

        cout << "world coordinates: " << xyz_world << endl;

        cout << "robo coordinates: " << helper.tavolo_to_robo(xyz_world) << endl;
        cout << "---------------------------------------------";
        
    }
    */
	//js=joint;
    //print_position();
}

void chatterCallback(const my_vision_messages::BlockList &b){
  ROS_INFO("block info \n");
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
    ros::Subscriber sub_js = n.subscribe("/vision_results", 1, block_listener_callback);

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
    thread robot_thread(call_robot, argc,argv); //fa partire la classe robot in un thread così può fare spin all'infinito
    //thread chatter_thread(start_chatter, argc,argv);
    thread block_listener_thread(start_block_callback, argc,argv);
    Robot robot(false, argc,argv);
    sensor_msgs::JointState joint;
    //essenziali
    while(!Robot::started){
        sleep(0.51);//aspetto di avere un feedback dal robot
    }
    cout << "robot, ready\n ";
    cout << "robot pos: \n";
    Robot::print_position(Robot::joint);

    float f=0;
    float delta;
    
    string s;
    kin.eval_ik_index(robot.j_to_q(Robot::joint));
    cout << "\n------------------------------------------------------------\ninizio... waiting for vision\n" ;

    vector<my_vision_messages::BlockList> bl_list;
    int bl_index=block_call_index;

    while(true){
        //cout << "cin per stalling... 0 per muove il braccio, altrimenti un valore per il gripper";
        //cin >> f;//f float

        //help.fill_pr_next(pr_f);
        if(bl_index != block_call_index){
            //se ho nuovi messaggi
            bl_list.push_back(BlockList_msg);
            cout << "pushing back new messages\n";
        }
        if(bl_list.size() > 3)//se è troppo grande tolgo il primo
            bl_list.erase(bl_list.begin());
        else
            continue;//salta se

        if(bl_list[0]==bl_list[1] && bl_list[0]==bl_list[2]){
            cout << "foud match...\n";
            procedure(robot,bl_list[2]);
        }else
            procedure(robot,bl_list[2]);
            cout << "messaggi diversi\n";
        
        //cout << BlockList_msg << endl;
        //cout << "moving to" << pr_f << endl;
        //cout << "joint j_to_q:" << robot.j_to_q(Robot::joint) << endl;
        //cout << "robot::joint: " << Robot::joint;
        
        sleep(1);
    }
   
    return 0;
}


/*! @brief starts the procedure onche a blocklist message has been provided \n 
*
* one run, has a few options to choose from: \n
* 0 for a testign procedure \n
* 1 to feed manual positions with pr_next \n
* 2 to start the robot procedure \n
*
* @param robot 
* @param my_vision_messages::BlockList list of blocks detected
*/
int procedure(Robot robot, my_vision_messages::BlockList bl){

    

    Kin kin;

    system("clear");

    cout << "procedure for:\n" << bl;
    cout << "\n-----------------------------------------\n";
    cout << "press a button to start (0 to test, 1 to set manual positions, 2 to start)" << endl;
    float f=0;
    cin >>f;

    if(f==0)
        testing(robot);
    if(f==1)
        feed(robot);

    double steps = 1500;
    double sleep_time=0.5;

    double h_safe = 0.65;
    double h_active = 0.74;
    Eigen::Vector < double, 6 > pr_safe;
    Eigen::Vector < double, 6 > q_safe;
    Eigen::Vector3d correction;

    
    pr_safe << 0, -0.35, 0.55, 0, 0, 0;
    correction << 0.014, -0.004,0;
    

    kin.eval_ik_index(robot.j_to_q(Robot::joint));
    robot.move_to(pr_safe,steps,f,false);
    kin.compute_fc(robot.j_to_q(Robot::joint));//q di safe
    sleep(1);
    robot.publish_grip(0);
    cout << "starting\n";

    for(int i_block=0; i_block< bl.blocks.size(); i_block++){

        Eigen::Vector3d p_theo;
        Eigen::Vector3d p_real;
        Eigen::Vector3d tmp3d;

        Eigen::Vector < double, 6 > q;
        Eigen::Vector < double, 6 > pr_i;
        Eigen::Vector < double, 6 > pr_f;


        //pr_i = helper.decode_final_pos(bl.blocks[i_blocks].class_number)
        tmp3d << bl.blocks[i_block].world_point.x,   bl.blocks[i_block].world_point.y,    bl.blocks[i_block].world_point.z;
        cout << "\nposizione mondo: n" << tmp3d;
        cout << "\nposizione mondo corretto: n" << tmp3d+correction;
        tmp3d=helper.tavolo_to_robo(tmp3d);
        tmp3d+=(correction+helper.get_extra_correction(bl.blocks[i_block].class_number));
        pr_f <<  tmp3d(0),tmp3d(1),h_safe,0,0,0;
        cout << "\nposizione robot: n" << kin.pr_to_p(pr_f) << endl;
        cout << "\nrotazione: " << bl.blocks[i_block].rot_angle*M_PI*2/360;
        cout << "\n----------------------------------------------\n";

    }

    cout << "\ncin per stalling\n";
    cin >> f;

    

    for(int i_block=0; i_block< bl.blocks.size(); i_block++){

        Eigen::Vector3d p_theo;
        Eigen::Vector3d p_real;
        Eigen::Vector3d tmp3d;
        Eigen::Vector3d extra_correction;

        Eigen::Vector < double, 6 > q;
        Eigen::Vector < double, 6 > pr_i;
        Eigen::Vector < double, 6 > pr_f;

        float grip_closed = -0.8;
        float grip_open = 0.8;
        float extra_h=0;

        kin.eval_ik_index(robot.j_to_q(Robot::joint));
        //pr_i = helper.decode_final_pos(bl.blocks[i_blocks].class_number);

        cout << "\ngoing for block:\n" <<  bl.blocks[i_block] << endl << endl;

        robot.publish_grip(grip_open);//apro
        sleep(sleep_time);
        robot.publish_grip(grip_closed);//chiudo
        sleep(sleep_time);
        robot.publish_grip(grip_open);//apro
        sleep(sleep_time);
        //xyz
        tmp3d << bl.blocks[i_block].world_point.x,   bl.blocks[i_block].world_point.y,    bl.blocks[i_block].world_point.z;
        tmp3d=helper.tavolo_to_robo(tmp3d);
        tmp3d+=correction;
        tmp3d+=helper.get_extra_correction(bl.blocks[i_block].class_number);
        pr_f <<  tmp3d(0),tmp3d(1),h_safe,0,0,0;
        //cout << "\nrotazione: " << bl.blocks[i_block].rot_angle*M_PI*2/360;
        pr_f(3)=bl.blocks[i_block].rot_angle*M_PI*2/360;//ruoto per ee
        //cout << "\nvado al  pezzo: " << pr_f;

        robot.move_to_shoulder(pr_f,steps,f,false);
        robot.move_to(pr_f,steps,f,false);//vado al pezzo 
        //robot.rotate(bl.blocks[i_block].rot_angle/360*M_PI);
        sleep(sleep_time);

        //cout << "\npubblico il gripper: ";
        robot.publish_grip(grip_open);
        sleep(sleep_time);

        extra_h=helper.get_extra_h(bl.blocks[i_block].class_number);
        pr_f(2) =h_active+extra_h;
        robot.move_to(pr_f,steps,f,false);//abbasso
        sleep(sleep_time*5);

        //cout << "\npubblico il gripper: ";
        robot.publish_grip(grip_closed);
        sleep(0.5);

        pr_f(2) =h_safe;
        robot.move_to(pr_f,steps,f,false);//alzo
        sleep(sleep_time);

        //cout << "\nevito clipping: ";
        //robot.move_to_shoulder(pr_f,3000,f,true);
        robot.move_to(pr_safe,steps,f,false);//per evitare clipping
        sleep(sleep_time);
        pr_f=helper.decode_final_pos(bl.blocks[i_block].class_number);
        pr_f(2) =h_safe;

        robot.move_to_shoulder(pr_f,steps,f,false);
        robot.move_to(pr_f,steps,f,false);//vado in base
        sleep(sleep_time);

        pr_f(2) =h_active;
        robot.move_to(pr_f,steps,f,false);//abbasso in base
        sleep(sleep_time);

        //cout << "\npubblico il gripper: ";
        robot.publish_grip(grip_open);
        sleep(sleep_time);

        pr_f(2) =h_safe;
        robot.move_to(pr_f,steps,f,false);//alzo
        sleep(sleep_time*5);

        //robot.move_to_shoulder(pr_safe,3000,f,true);
        robot.move_to(pr_safe,steps,f,false);
        cout << "\nfinito: " << i_block << endl;
        //cout << "pausa... cin per continuare";
        //cin >> f;
        //sleep(3);
    }

    cout << "\nfinito tutto\n";
    return 0;

}

int testing(Robot robot){
    cout << "\ntesting....\napro il gripper\n";
    robot.publish_grip(0.6);
    sleep(3);
    cout << "\nchiudo\n";
    robot.publish_grip(-0.1);
    sleep(3);
    int f=0;

    Eigen::Vector < double, 6 > pr_f;
    pr_f << 0, -0.3, 0.45, 0, 0, 0;
    
    cout << "cambio 3\n";
    pr_f(3)=1;
    robot.move_to(pr_f,1000,f,false);
    pr_f(3)=0;
    sleep(3);

    /*
    cout << "\nsafe\n";
    
    robot.move_to(pr_f,1000,f,false);
    sleep(3);

    

    cout << "cambio 4\n";
    pr_f(4)=1;
    robot.move_to(pr_f,1000,f,false);
    pr_f(4)=0;
    sleep(3);

    cout << "cambio 5\n";
    pr_f(5)=1;
    robot.move_to(pr_f,1000,f,false);
    pr_f(5)=0;
    sleep(3);

    cout << "\nafe\n";
    robot.move_to(pr_f,1000,0,false);
    sleep(3);
    */
}

int feed(Robot robot){
    float f;
    float g=0;
   
    while(true){
        cout << "\n in feeding pr_next 0 per gripper, 1 per vuoversi  resto per uscire\n";
        cin >> f;
        if(f==0){
            cout << "gripper value: ";
            cin >> g;
            robot.publish_grip(g);
        }else if(f==1){
            Eigen::Vector < double, 6 > pr_f;
            helper.fill_pr_next(pr_f);
            robot.move_to(pr_f,1000,0,false);
        }else
            break;//esci dal while
    }

    return f;
}
