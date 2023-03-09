
#ifndef HELPER__CPP
#define HELPER__CPP

#pragma once

#include "std_msgs/String.h"
#include <unistd.h> 
#include <stdio.h> 
#include <fcntl.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include "Eigen/Eigen/Dense"
#include <cmath>
#include "my_vision_messages/Pointxyz.h"
#include "my_vision_messages/Block.h"
#include "my_vision_messages/BlockList.h"

#define BL_DIFF_RANGE 0.03
#define BL_DIFF_CONFIDANCE 0.05

//#include "robot.cpp"



using namespace std;

/**
@file helper.cpp
@brief helper function for different applications
*/

/*! @brief helper function for different applications
*
*/

class Helper{
    public:
        Helper(){

        }

        float** sin_square(float start[], float end[], int steps){

            float** path =0;
            path   = new float*[steps];//altezza = steps

            for(int i =0;i < steps ; i++){
                path[i]= new float[6]; //larghezza = 6 per le coordinate
                //cout << "created: " << i << endl;
                for (int j = 0; j < 6; j++){
                    // q_i+ (math.sin((math.pi/2)*i/steps)**2)*diff
                    //cout << "created: " << i << " " << j << endl;
                    path[i][j]=start[j]+pow(sin((M_PI/2*i/steps)),2)*(end[j]-start[j]);
                    
                    //cout << "inserito: " << path[i][j] << endl;
                }

            }

            cout << "creato" << endl << endl;

            return path;
        };


        int fill_pr_i_f(Eigen::Vector < double, 6 > & pr_i, Eigen::Vector < double, 6 > & pr_f){

            std::ifstream myfile ("src/locosim/test_cpp/src/master_positions.txt");

            if(!myfile){
                cout << "\nmanca master_positions.txt\n";
                exit(-1);
            }

            float f[6];
            for(int i=0; i < 6; i++){
                myfile >> f[i];
                cout << f[i] << ", ";
            }
            pr_i << f[0],f[1],f[2],f[3],f[4],f[5];
            for(int i=0; i < 6; i++){
                myfile >> f[i];
                cout << f[i] << ", ";
            }
            pr_f << f[0],f[1],f[2],f[3],f[4],f[5];

            cout << "iniziale: " << pr_i << endl << endl;
            cout << "finale: " << pr_f << endl << endl;

            myfile.close();
            return 0;
        }
    /*! @brief debugging method to manually input positions for the robot \n 
*
* reads from src/locosim/test_cpp/src/pr_next.txt \n
* only reads the first 6 lines
*
* @param pr_f reference to the vector to store what's read
*/

        int fill_pr_next(Eigen::Vector < double, 6 > & pr_f){

            std::ifstream myfile ("src/locosim/test_cpp/src/pr_next.txt");

            if(!myfile){
                cout << "\nmanca pr_next.txt\n";
                exit(-1);
            }

            float f[6];
            for(int i=0; i < 6; i++){
                myfile >> f[i];
                //cout << f[i] << ", ";
            }
            pr_f << f[0],f[1],f[2],f[3],f[4],f[5];
            cout << "finale: " << pr_f << endl << endl;
            myfile.close();

            return 0;
        }

        void print_pr(Eigen::Vector < double, 6 > & pr_i){
            cout << "todo" << endl;
        }

        Eigen::MatrixXd fillK(int param = 1){
            Eigen::MatrixXd k = Eigen::MatrixXd(6,6);

            k<< 1,	0,	0,	0,	0,	0,
                0,	1,	0,	0,	0,	0,
                0,	0,	1,	0,	0,	0,
                0,	0,	0,	1,	0,	0,
                0,	0,	0,	0,	1,	0,
                0,	0,	0,	0,	0,	1;
            k=k*param;

        return k;
    }

/*! @brief calculates the distance between 2 positions in space \n 
*
* takes eigen vectors of 6
*
* @see dist(Eigen::Vector3d pr_i,Eigen::Vector3d pr_f)
*
* @return float distance in absolute value 
*/

    float dist(Eigen::Vector < double, 6 >pr_i,Eigen::Vector < double, 6 > pr_f){
        return (    pow(pr_i[0],2) + pow(pr_i[1],2) + pow(pr_i[2],2)    ) - (    pow(pr_f[0],2) + pow(pr_f[1],2) + pow(pr_f[2],2)    );
    }
/*! @brief calculates the distance between 2 positions in space \n
*
* takes eigen vectors of 3
*
* @see dist(Eigen::Vector < double, 6 >pr_i,Eigen::Vector < double, 6 > pr_f)
* @return float distance in absolute value 
*/
    float dist(Eigen::Vector3d pr_i,Eigen::Vector3d pr_f){
        return (    pow(pr_i[0],2) + pow(pr_i[1],2) + pow(pr_i[2],2)    ) - (    pow(pr_f[0],2) + pow(pr_f[1],2) + pow(pr_f[2],2)    );
    }

    static float abs(Eigen::Vector < double, 6 >pr_i){

        return std::abs(    pow(pr_i[0],2) + pow(pr_i[1],2) + pow(pr_i[2],2)    );
    }

    
    Eigen::Vector3d cam_to_world(Eigen::Vector3d camera){
        //camera x y depth
        Eigen::Vector3d world;

        Eigen::Matrix3d wrc;
        Eigen::Vector3d base_offset;
        Eigen::Vector3d xc;
        wrc <<   0,      -0.49948,  0.86632,
                -1,         0,          0,
                -0,     -0.86632, -0.49948;
        base_offset << 0.5,  0.35, 1.75;
        xc << -0.9,0.24,-0.35;

        world = wrc*camera + xc + base_offset;

        return world;

    }
/*! @brief converts world frame fo robot frame coordinates
*
* @param camera x,y,z coordinates of the world frame
* @return float distance in absolute value 
*/
    Eigen::Vector3d tavolo_to_robo(Eigen::Vector3d world){
        //camera x y depth
        Eigen::Vector3d robo;
        robo(0)=world(0)-0.5;//x tavolo
        robo(1)=-world(1)+0.35;         //y tavolo
        robo(2)=world(2);

        return robo;
    }

    
    double constrainAngle(double x){
        x = fmod(x,M_PI*2);
        if (x < 0)
            x += M_PI*2;
        return x;

    }
/*! @brief wrapps differences in angles
* @fn dist_constrain(double x)
* @see dist_constrain(Eigen::Vector<double, 6> q)
* @param x angle
* @return wrapped angle difference
*/
    double dist_constrain(double x){
        
        if (x > M_PI)//wrapping
            return std::abs(M_PI*2-x);
        return x;
    }
/*! @brief constrains angle to [-180,180]
* @fn constrainAngle180(double x)
*   @see constrainAngle180(Eigen::Vector<double, 6> q)
* @param x angle
*/
    double constrainAngle180(double x){//da -180 a 180
        x = fmod(x,M_PI*2);//normalizzo
        if (x < -M_PI)
            return x+ M_PI*2;
        if (x > M_PI)
        return x - M_PI*2;
    }
/*! @brief constrains angle to [-360,360]
* @fn constrainAngle720(double x)
*   @see constrainAngle720(Eigen::Vector<double, 6> q)
* @param x angle
*/
    double constrainAngle720(double x){//cambio da + a - e viceversa
        //x = fmod(x ,M_2_PI);
        if (x < 0)
            return x + M_PI*2;//se - diventa +
        return x - M_PI*2;//se + diventa -
    }

    Eigen::Vector<double, 6> constrainAngle(Eigen::Vector<double, 6> q){
        Eigen::Vector<double, 6> ret;
        for(int i=0; i< 6; i++){
            ret(i)=constrainAngle(q(i));
        }
        return ret;
    }

/*! @brief constrains angle vector to [-180,180]
* @fn constrainAngle180(Eigen::Vector<double, 6> q)
*   @see constrainAngle180(double x)
* @param q vector of 6 angles
*/
    Eigen::Vector<double, 6> constrainAngle180(Eigen::Vector<double, 6> q){
        Eigen::Vector<double, 6> ret;
        for(int i=0; i< 6; i++){
            ret(i)=constrainAngle180(q(i));
        }
        return ret;
    }
/*! @brief constrains angle vector to [-360,360]
* @fn constrainAngle720(Eigen::Vector<double, 6> q)
*   @see constrainAngle720(double x)
* @param q vector of 6 angles
*/
    Eigen::Vector<double, 6> constrainAngle720(Eigen::Vector<double, 6> q){
        Eigen::Vector<double, 6> ret;
        for(int i=0; i< 6; i++){
            ret(i)=constrainAngle720(q(i));
        }
        return ret;
    }
/*! @brief wrapps differences in angles
* @fn dist_constrain(Eigen::Vector<double, 6> q)
* @see dist_constrain(double x)
* @param q vector of 6 angles 
* @return wrapped angle difference vector
*/

    Eigen::Vector<double, 6> dist_constrain(Eigen::Vector<double, 6> q){
        Eigen::Vector<double, 6> ret;
        for(int i=0; i< 6; i++){
            ret(i)=dist_constrain(q(i));
        }
        return ret;
    }

    void print_BlockList(my_vision_messages::BlockList& bl){
        cout << bl;
    }

    Eigen::Vector<double, 6> decode_final_pos(string s){
        float cls = stof(s);
        cout << "\nclass:" << cls;
        Eigen::Vector3d pos;
        switch((int)cls){
            case 0:
                pos << 0.92, 0.28, 0.9;
                break;
            case 1:
                pos << 0.92, 0.41, 0.9;
                break;
            case 2:
                pos << 0.92, 0.54, 0.9;
                break;
            case 3:
                pos << 0.92 , 0.67, 0.9;
                break;
            case 4:
                pos << 0.81, 0.28 , 0.9;
                break;
            case 5:
                pos << 0.81, 0.41, 0.9;
                break;
            case 6:
                pos << 0.81, 0.54, 0.9;
                break;
            case 7:
                pos << 0.81, 0.67, 0.9;
                break;
            case 8:
                pos << 0.70, 0.28, 0.9;
                break;
            case 9:
                pos << 0.70, 0.41, 0.9;
                break;
            case 10:
                pos << 0.70, 0.54, 0.9;
                break;
            default:
                pos << 0.7 , 0.67, 0.9;
                break;
        }

        Eigen::Vector3d tmp = tavolo_to_robo(pos);
        Eigen::Vector<double,6> ret;
        ret << tmp(0),tmp(1),tmp(2),0,0,0;

        return ret;
    }

    
    Eigen::Vector3d get_extra_correction(string s){
        float cls = stof(s);
        //cout << "\nclass:" << cls;
        Eigen::Vector3d extra;
        extra << 0,0,0;
        switch((int)cls){
            case 1:
                extra(0)=-0.008;
                break;
            case 9:
            case 10:
                extra(0)=0.008;
                break;
            default:
                break;
        }

        return extra;
    }

    float get_extra_h(string s){
        float cls = stof(s);
        //cout << "\nclass:" << cls;
        float extra=0;
        switch((int)cls){
            case 2:
            case 7:
                extra=0.03;
                break;
            default:
                break;
        }

        return extra;
    }


};

ostream& operator<<(ostream& os, const my_vision_messages::Pointxyz& p){
        os << "x: " << p.x << "\ty: " << p.y << "\tz: " << p.z;
        return os;
}

ostream& operator<<(ostream& os, const my_vision_messages::BlockList& bl){
    if(bl.blocks.size()<1){
        os<< "BlockList vuota";
        return os;
    }
    for(int i= 0; i< bl.blocks.size(); i++){
        os << "class number: " << bl.blocks[i].class_number << "\n";
        os << "point camera: " << bl.blocks[i].point << "\n";
        os << "rotation angle: " << bl.blocks[i].rot_angle << "\n";
        for(int j=0; j< bl.blocks[i].top_btm_l_r.size(); j++){
            os << "\t" << bl.blocks[i].top_btm_l_r[j] << "\n";
        }
            os << "up_dw_stand_lean: " << bl.blocks[i].up_dw_stand_lean << "\n";
            os << "confidance: " << bl.blocks[i].confidence << "\n";
            os << "world frame point: " << bl.blocks[i].world_point << "\n";
    }
    return os;
}

bool operator==( const my_vision_messages::BlockList& bl1, const my_vision_messages::BlockList& bl2 ){
    if(bl1.blocks.size()!=bl2.blocks.size() || bl1.blocks.size()==0 || bl2.blocks.size()==0)
        return false;
    for(int i=0; i< bl1.blocks.size(); i++){
        if(bl1.blocks[i].class_number == bl2.blocks[i].class_number
        && bl1.blocks[i].world_point.x - bl2.blocks[i].world_point.x < BL_DIFF_RANGE
        && bl1.blocks[i].world_point.y - bl2.blocks[i].world_point.y < BL_DIFF_RANGE
        && bl1.blocks[i].world_point.z - bl2.blocks[i].world_point.z < BL_DIFF_RANGE
        && bl1.blocks[i].confidence - bl2.blocks[i].confidence < BL_DIFF_CONFIDANCE)
        //ok
            int a=0;
        else return false;
    }
    return true;
    
}

#endif