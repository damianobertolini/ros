
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
#include "Helper.cpp"
#include <iostream>
#include <fstream>
#include <string>
#include "Kin.cpp"

using namespace std;

int main(int argc, char ** argv) {
	
    Kin kin;
    
    Eigen::Vector <double, 6> pos;
    pos << -0.165687,
-0.120752,
 0.940965,
  2.37135,
-0.756831,
 -2.64711;
    Eigen::Vector <double, 6> pos2;
    pos2 << 0.5, 0.1, 0.5, M_PI/6, M_PI/3, M_PI/4;
    Eigen::Vector <double, 6> pos3 = kin.compute_ik(pos)[0];
    Eigen::Matrix3d rotm=kin.eul2rotXYZ(kin.pr_to_r(pos));
    cout << "xyz: \n" << rotm <<  endl ;

    rotm=kin.eul2rotZYX(kin.pr_to_r(pos));
    cout << "zyx: \n" <<  rotm <<  endl ;

    cout << "\n------------------------------------\n";

    pos3 = kin.compute_ik(pos)[0];
    //vector<Eigen::Vector < double, 6 >> ik = kin.compute_ik(pos);
    cout << "pos3:" << pos3;
    cout << "\n------------------------------------\n";
    cout << "\n------------------------------------\n";
    cout << "\n------------------------------------\n";
    /*
    std::vector<Eigen::Vector < double, 6 >> p2p = kin.p2p(pos,pos2,30);

    for(int i=0; i< 30 ;i++){
        cout << "\n"  << i << ") " << p2p[i] << endl << endl;
    }
    */



    cout << "\n------------------------------------\n";

    Eigen::Vector <double , 6> A = Eigen::Vector <double , 6> (0, -0.425, -0.3922, 0, 0, 0);//distanze su A


    double p41xz_1 = 0.930377;

    double p41xz_2 = 0.765861;

    double p41xz_3 = 0.892802;

    double p41xz_4 = 0.809351;


    double th3_1 = real(acos((pow(p41xz_1,2)-pow(A(1),2)-pow(A(2),2))/(2*A(1)*A(2))));
    double th3_2 = real(acos((pow(p41xz_2,2)-pow(A(1),2)-pow(A(2),2))/(2*A(1)*A(2))));
    double th3_3 = real(acos((pow(p41xz_3,2)-pow(A(1),2)-pow(A(2),2))/(2*A(1)*A(2))));
    double th3_4 = real(acos((pow(p41xz_4,2)-pow(A(1),2)-pow(A(2),2))/(2*A(1)*A(2))));

    cout << "th3_1 : " << th3_1 << endl;
    cout << "p41xz_1 " << p41xz_1 <<  endl << endl;
    cout << "th3_2 : " << th3_2 << endl;
    cout << "p41xz_2 " << p41xz_2 <<  endl<< endl;
    cout << "th3_3 : " << th3_3 << endl;
    cout << "p41xz_3 " << p41xz_3 <<  endl<< endl;
    cout << "th3_4 : " << th3_4 << endl;
    cout << "p41xz_4 " << p41xz_4 <<  endl<< endl;

    cout << "\n------------------------------------\n";

    double tmp1 = (pow(p41xz_1,2)-pow(A(1),2)-pow(A(2),2))/(2*A(1)*A(2));

    cout << tmp1 <<  endl  << endl;

    cout << "\nacos tmp1: " << kin.safe_acos(tmp1);

    cout << "\nacos -tmp1: " << kin.safe_acos(-tmp1);

    cout << "\natan2(-12,0) " << atan2(-12,0);



    return 0;
    

}
