
#ifndef HELPER__CPP
#define HELPER__CPP

#include "std_msgs/String.h"
#include <unistd.h> 
#include <stdio.h> 
#include <fcntl.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <cmath>

//#include "robot.cpp"

using namespace std;



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

            std::ifstream myfile ("master_positions.txt");
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

        int fill_pr_next(Eigen::Vector < double, 6 > & pr_f){

            std::ifstream myfile ("pr_next.txt");
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

    

    float dist(Eigen::Vector < double, 6 >pr_i,Eigen::Vector < double, 6 > pr_f){
        return (    pow(pr_i[0],2) + pow(pr_i[1],2) + pow(pr_i[2],2)    ) - (    pow(pr_f[0],2) + pow(pr_f[1],2) + pow(pr_f[2],2)    );
    }
    float dist(Eigen::Vector3d pr_i,Eigen::Vector3d pr_f){
        return (    pow(pr_i[0],2) + pow(pr_i[1],2) + pow(pr_i[2],2)    ) - (    pow(pr_f[0],2) + pow(pr_f[1],2) + pow(pr_f[2],2)    );
    }

    static float abs(Eigen::Vector < double, 6 >pr_i){

        return std::abs(    pow(pr_i[0],2) + pow(pr_i[1],2) + pow(pr_i[2],2)    );
    }


        
};

#endif