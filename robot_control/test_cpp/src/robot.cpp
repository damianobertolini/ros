#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
//#include <std_msgs/msg/float64_multi_array.hpp>
#include "Eigen/Eigen/Dense"

#include <sstream>
#include <iostream>
#include <string>
#include <thread>
#include "Helper.cpp"
#include "Kin.cpp"


#define N_GRIP 2

using namespace std;


void robot_msg_callback(const std_msgs::String::ConstPtr& message);


void js_callback(const sensor_msgs::JointState& joint);




class Robot{
    public: 

        static inline sensor_msgs::JointState joint;
        static inline std::vector<double> gripper_j;
        static inline std_msgs::String msg;
        static inline int stat=0;
        static inline bool started = false;
        static inline bool moving = false;
        //ros::init(argc, argv,"talker");
        static inline ros::Publisher js_pub;
        static inline int n_grip = 0;
    
        Robot(){
            //vuoto
        };

        Robot(bool init,int argc, char ** argv){//da chiamare una volta per non avere errori
            if(init){
                joint.position={0.0,0.0,0.0,0.0,0.0,0.0};
            }
            int i=1;
            ros::init(argc,argv, "robot_cpp");
            ros::NodeHandle n;
            //ros::Publisher js_pub = n.advertise < sensor_msgs::JointState > ("/command", 1);
            //Robot::js_pub = n.advertise < sensor_msgs::JointState > ("/command", 1);
            //Robot::js_pub = n.advertise < sensor_msgs::JointState > ("/ur5/joint_group_pos_controller/command", 1);
            Robot::js_pub = n.advertise < std_msgs::Float64MultiArray > ("/ur5/joint_group_pos_controller/command", 1);
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
            ros::init(argc, argv, "robot_cpp_callback");
            ros::NodeHandle n;
            ros::Rate loop_rate(1);

            //ros::AsyncSpinner spinner(0);
            //spinner.start();
            //sub = n.subscribe("/ur5/joint_states", 1, js_callback);
            //ros::Subscriber sub = n.subscribe("chatter", 10, robot_msg_callback);
            ros::Subscriber sub_js = n.subscribe("/ur5/joint_states", 1, js_callback);

            cout << "start spinning robot" << endl;
            ros::MultiThreadedSpinner spinner(2);
            spinner.spin();
            cout << "done spinning robot" << endl;
        }

        

        int publish(Eigen::Vector < double, 6 > th, bool fill = true, double gripper = 0){
            
            std_msgs::Float64MultiArray f64j;
            sensor_msgs::JointState joint_tmp;

            //std::cout << "\npublishing...\n" << std::endl;

             f64j.data.empty();

            for(int i=0; i<6;i++){
                //joint.position[i] = th[i];
                //std::cout << "\npublishing... " << th[i] << std::endl;
                f64j.data.push_back(th[i]);
                //std::cout << "\nstacked..." << std::endl;
            }

            //f64j.data.push_back(gripper);
            //f64j.data.push_back(gripper);
            
            //per soft gripper

            for(int i=0; i< Robot::n_grip;i++){ //pos 1 2 (3)
                //f64j.data.insert ( std::next(f64j.data.begin()) , gripper );//posizione 2
                f64j.data.push_back( Robot::gripper_j[i] ); // attacco lo stato corrente del gripper
            }

            //std::cout << "argc: " << argc << "\nargv: " << argv << std::endl;
            //std::cout << "joint to publish: " << joint << std::endl;
            js_pub.publish(f64j);

            //std::cout << "\published!!\n" << std::endl;

            return 0;
        }

        int publish_grip(double gripper = 0){
            
            std_msgs::Float64MultiArray f64j;
            sensor_msgs::JointState joint_tmp;

            f64j.data.empty();

            for(int i=0; i<6;i++){
                f64j.data.push_back(Robot::joint.position[i]);//pubblico la stessa posizione
            }

            print_f64j(f64j);

            cout << "\nn_grip: " << Robot::n_grip << endl;

            for(int i=0; i < Robot::n_grip ; i++){ //pos [6] [7] [8]

                if(Robot::gripper_j.empty())
                    cout << "\ngripper vuoto\n";
                cout << "\nset gripper to: "<< gripper <<"\n";
                //f64j.data.insert ( std::next(f64j.data.begin()) , gripper );//posizione [1]
                //f64j.data.push_back( double(rand()%int(M_PI*100))/100 -  (M_PI/2) );
                f64j.data.push_back( gripper );
            }

            print_f64j(f64j);

            js_pub.publish(f64j);

            return 0;
        }

        void print_f64j(std_msgs::Float64MultiArray f64j){
            for(int i=0; i< f64j.data.size();i++){
                cout << "\nf64j.data[" << i << "] " << f64j.data[i];
            }
            cout << "\n\n";
        }
        /*
        int publish(sensor_msgs::JointState& joint_2, bool fill = true, double gripper = 0){

            std_msgs::Float64MultiArray f64j;
            Eigen::Vector < double, 6 > th;

            //if(fill){
            if(false){
                //std::cout << "filling with 0" << std::endl;
                joint_2.velocity={0.0,0.0,0.0,0.0,0.0,0.0};
                joint_2.effort={0.0,0.0,0.0,0.0,0.0,0.0};
            }

            for(int i=0; i<6;i++){
                th(i) = joint.position[i];
            }

            //std::cout << "argc: " << argc << "\nargv: " << argv << std::endl;
            //std::cout << "joint to publish: " << joint << std::endl;
            //js_pub.publish(joint_2);

            publish(th,fill);

            return 0;
        }
        */

        float move_to(Eigen::Vector < double, 6 > pr_f, int steps = 3000, float k_coeff=0.01, int fix=false){
            Helper help;
            Eigen::MatrixXd k = Eigen::MatrixXd::Identity(6,6) * k_coeff;
            sensor_msgs::JointState j_now = joint;
            Kin kin;
            float f;
            Eigen::Vector < double, 6 > pr_i;//stato attuale
            //Eigen::Vector < double, 6 > pr_f;

            //help.fill_pr_i_f(pr_i,pr_f);//prendo il punto finale
            Eigen::Vector < double, 6 > q = j_to_q(j_now);//punto iniziale
            //q = help.constrainAngle180(q);//180-180
            
            kin.compute_fc(q);
            pr_i=kin.get_pr_now();
            /*
            pr_i(0) = kin.get_ee_p()[0];
            pr_i(1) = kin.get_ee_p()[1];
            pr_i(2) = kin.get_ee_p()[2];
            pr_i(3) = kin.rotm2eul(kin.T0e)[0];
            pr_i(4) = kin.rotm2eul(kin.T0e)[1];
            pr_i(5) = kin.rotm2eul(kin.T0e)[2];//sovrascrivo con la posizione attuale
            */
            

            cout << "moving from: " << pr_i << endl << endl;
            cout << "to: " << pr_f << endl << endl;

            //vector<Eigen::Vector < double, 6 >> path_theory = kin.fillpath(q,pr_f, steps);
            //vector<Eigen::Vector < double, 6 >> path = kin.da_a(path_theory,q,k,steps);
            vector<Eigen::Vector < double, 6 >> path = kin.p2p(pr_i,pr_f,steps);
            /*
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
            path[0]=j_to_q(joint);
            cout << "joints attuali:" << q << endl<<endl;
            cout << "path[0]" << path[0] << endl;
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
            */
            ofstream myfile;
            /*
            
            myfile.open ("path_p_theo.txt");
            for (Eigen::Vector < double, 6 > i: path_theory)
                    myfile << i(0)<< "," <<i(1)<< "," <<i(2)<< "," <<i(3)<< "," <<i(4)<< "," <<i(5)<< "\n";
            myfile.close();
            */
            
            myfile.open ("path.txt");
            for (Eigen::Vector < double, 6 > i: path)
                    myfile << i(0)<< "," <<i(1)<< "," <<i(2)<< "," <<i(3)<< "," <<i(4)<< "," <<i(5)<< "\n";
            myfile.close();
            
            //cout << "\npath dim: "<< path.size()<<"\n";

            ros::Rate loop_rate(1000);
            int path_i=0;

            while(ros::ok && path_i<steps){
        
                //sleep(0.001);
                //cout << path_i << ")\n" << std::flush;;

                //Robot::print_position(Robot::joint);
                //cin >> f;
                publish(path[path_i]);
                //path_i=steps;
                //ros::sleep(0.2);
                //cout << path_i << "\t";
                path_i++;
                //cout << "loop" << path_i << " ";
                loop_rate.sleep();
                //sleep(0.001);
            }

            sleep(0.3);//a caso

            if(fix){
                cout << "\n cin per fix: \n";
                cin >> f;
                q = j_to_q(joint);//dove sono alla fine della routine
                kin.compute_fc(q);
                pr_i(0) = kin.get_ee_p()[0];
                pr_i(1) = kin.get_ee_p()[1];
                pr_i(2) = kin.get_ee_p()[2];
                pr_i(3) = kin.rotm2eul(kin.T0e)[0];
                pr_i(4) = kin.rotm2eul(kin.T0e)[1];
                pr_i(5) = kin.rotm2eul(kin.T0e)[2];

                Eigen::Vector < double, 6 > delta = pr_f-pr_i;//quanto manca al target teorico

                //move_to(pr_f+delta,steps,k_coeff,false);

            }
            

            print_position();


            cout << "\nsteps fatti: " << path_i << endl;
            return 0;
        }

        Eigen::Vector < double, 6 > j_to_q(sensor_msgs::JointState j_tmp){
            Eigen::Vector < double, 6 > q_tmp;
            for(int i=0; i< 6; i++)
                q_tmp[i]=j_tmp.position[i];
            return q_tmp;
        }

        //sensor_msgs::JointState& get_latest_js(){
        //    return 0;
        //}
        /*
        int set_gripper(double grip_d = 0){

            sensor_msgs::JointState j_tmp = Robot::joint;
            Robot::gripper_j.clear();

            for(int i=0; i < Robot::n_grip; i++){

                j_tmp.position[i+1] = grip_d; //da [1] in poi
                cout << "\nset: " << grip_d << " for gripepr " << i+1 << std::endl;

            }

            //publish(j_tmp);
        }
        */

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
        static int print_position(){
            return print_position(joint);
        };

};

//std_msgs::String Robot::msg = new std_msgs::String();




void robot_msg_callback(const std_msgs::String::ConstPtr& message){
    //cout << "in msg_callback" << endl;
    //ROS_INFO("I heard: [%f]", js->velocity.c_str());
    //ROS_INFO("msg");
	//ROS_INFO("I heard: [%s]", message->data.c_str());
    //cout << "msg: " << message->data.c_str() << endl;
    cout << "---------------------------------------------" << endl;
    Robot::stat++;
    //cout << "stat: " << Robot::stat << endl;
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

    //ROS_INFO("I heard: [%f]", j.velocity.c_str());
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
    Robot::joint = sensor_msgs::JointState(j);//
    Robot::n_grip=Robot::joint.position.size()-6;//6 per joint il resto per gripper

    Robot::gripper_j.clear();

    for(int i=0; i < Robot::n_grip; i++){

        Robot::gripper_j.push_back( Robot::joint.position[1] );   //da posizione 1 
        Robot::joint.position.erase(std::next(Robot::joint.position.begin(), 1)); //toglie il secondo elemento n volte

    }
    
    swap(Robot::joint.position[0],Robot::joint.position[2]); //perche si
    Robot::started = true;
    
    //print_position();
}

void swap(double& f1,double& f2){
    float buff;
    buff=f1;
    f1=f2;
    f2=buff;
}
