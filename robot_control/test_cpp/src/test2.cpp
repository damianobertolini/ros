
#include "std_msgs/String.h"
#include <unistd.h> 
#include <stdio.h> 
#include <fcntl.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Eigen/Dense>
#include "Helper.cpp"
#include <iostream>
#include <fstream>
#include <string>
#include "Kin.cpp"

using namespace std;

int main(int argc, char ** argv) {
	
    Kin kin;
    
    Eigen::Vector <double, 6> pos;
    pos << -0.42,
        -0.07,
        0.5,
        0,
        0,
        0;

    //Eigen::Vector <double, 6> pos2;
    //pos2 << 0.5, 0.1, 0.5, M_PI/6, M_PI/3, M_PI/4;
    //Eigen::Vector <double, 6> pos3 = kin.compute_ik(pos)[0];
    //Eigen::Matrix3d rotm=kin.eul2rotXYZ(kin.pr_to_r(pos));
    //cout << "xyz: \n" << rotm <<  endl ;

    //rotm=kin.eul2rotZYX(kin.pr_to_r(pos));
    //cout << "zyx: \n" <<  rotm <<  endl ;
    //cout << "\n------------------------------------\n";

    std::vector<Eigen::Vector < double, 6 >> ik = kin.compute_ik(pos);
    //vector<Eigen::Vector < double, 6 >> ik = kin.compute_ik(pos);
    /*
    std::vector<Eigen::Vector < double, 6 >> p2p = kin.p2p(pos,pos2,30);

    for(int i=0; i< 30 ;i++){
        cout << "\n"  << i << ") " << p2p[i] << endl << endl;
    }
    */
    /*
    cout << "ik for :" << pos << endl;

    for(int i=0; i<ik.size();i++){
        cout << i << ")\n";
        for(int j=0; j< 6; j++){
            cout << ik[i](j) << ",";
        }
        cout << "\n";
    }

    Eigen::Vector <double, 6> pos2;
    pos2 << -0.42,
        -0.07,
        15,
        17,
        M_PI*5,
        -86;
    Helper h;

    
    Eigen::Vector <double, 6> pos3;
    pos3 <<  -0.3200020652135178,-0.7800190541406344,-2.560051202291631,    -1.6300324485169746, -1.5700321358755218, 3.4900689818455577;
    

    cout << "angolo: " << pos3 <<  endl;

    cout << "\nhelper wrapper: " << h.constrainAngle(pos3) << endl;

    cout << "\nhelper wrapper1 : " << h.constrainAngle180(pos3) << endl;
    cout << "\nfine";


    
    kin.compute_fc(pos3);
    cout << "pos finale: " << kin.get_pr_now();
    */

    Eigen::Vector <double, 6> pos_j;
    pos_j <<  -0.3200020652135178,-0.7800190541406344,-2.560051202291631,    -1.6300324485169746, -1.5700321358755218, 3.4900689818455577;
    kin.compute_fc(pos_j);
    Eigen::Matrix4d rotm= kin.get_T0e();
    Eigen::Vector3d eul= kin.rotm2eul(rotm);

    cout << "\n\tjoints\n" << pos_j;
    cout << "\n\tpr_i: \n" << kin.get_pr_now();
    cout << "\n\trot matrix: \n" << rotm;
    cout << "\n\trotm 2 eul xyz:\n" << eul;
    cout << "\n\teul to xyz:\n" << kin.eul2rotXYZ(eul);
    cout << "\n\teul to zyx:\n" << kin.eul2rotZYX(eul);


    return 0;
    

}
