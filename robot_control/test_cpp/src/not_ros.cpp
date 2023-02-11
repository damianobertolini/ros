
#include "Kin.cpp"
#include <iostream>
#include "Helper.cpp"
#include <vector>
#include "Eigen/Eigen/Dense"
#include <iostream>
#include <fstream>

using namespace std;

int main(){

    Kin kin;
    
    //Eigen::Matrix4d get_T0e = kin.get_T0e(Th);

    //cout << "matrice: " << endl << get_T0e << endl;

    Helper h;
    float start[6]={1,1,1,1,1,1};
    float end[6]={2,2,2,2,2,2};
   
    float** matrix = h.sin_square(start,end,10);

    for(int a = 0; a < 10; a++){
        for(int b = 0; b < 6; b++){
            cout << matrix[a][b] << " ";
        }
        cout << endl;
    } 


    //Eigen::Vector<double, 6> Th(0.089159, 0, 0, 0.10915, 0.09465, 0.0823);

    //Eigen::Vector<double, 6> Th(0,-M_PI/2,0,0,0,0);
    //-0.69004, 0.04096, -1.65882,-0.85673, -0.8407,   0.
    //Eigen::Vector<double, 6> Th(-0.69004, 0.04096, -1.65882,-0.85673, -0.8407,   0);

    
/*
    kin.compute_fc(Th, false);
    ee= kin.get_ee_p();
    cout << "\nee iniziale1:\n" << ee;
    cout << "\nrot iniziale1:\n" << kin.get_T0e();


    Th = Eigen::Vector < double, 6 >(M_PI, -M_PI/2, 0.0, 0.0, -M_PI, 0.0);

    kin.compute_fc(Th, false);
    ee= kin.get_ee_p();
    cout << "\n\n\nee iniziale2:\n" << ee;
    cout << "\nrot iniziale1:\n" << kin.get_T0e();

    Eigen::Vector < double, 6 > Th2 = Eigen::Vector < double, 6 >(3.4362   ,-0.0416    ,2.3889    ,2.3650    ,1.5708   ,-1.8654);

    kin.compute_fc(Th2, false);
    ee= kin.get_ee_p();
    cout << "\n\n\nee iniziale2:\n" << ee;
    cout << "\nrot iniziale1:\n" << kin.get_T0e();

    

    cout << "\n----------------------------------------------" << endl;
    kin.compute_fc(Th);

    Eigen::Vector3d rpy2=kin.rotm2eul(kin.get_T0e());
    Eigen::Vector3d pos_rot= kin.get_rot_p();
    Eigen::Vector3d pos_ee= kin.get_ee_p();

    cout << "\nrpy iniziale:\n" << rpy2;
    cout << "\nrrot iniziale:\n" << pos_rot;
    cout << "\nq iniziale:\n" << Th;
    cout << "\nee iniziale:\n" << pos_ee;

    
    cout << "\n----------------------------------------------" << endl;

    vector<Eigen::Vector < double, 6 >> path = kin.compute_ik(pos_ee,rpy2,Th2,100, false);

    Eigen::Vector<double, 6> Thf;
    Thf=path[path.size()-1];

    cout << "\n----------------------------------------------" << endl;
    cout << "\n----------------------------------------------" << endl;
    cout << "\n----------------------------------------------" << endl;

    kin.compute_fc(Thf, false);

    Eigen::Vector3d rpy3=kin.rotm2eul(kin.get_T0e());
    Eigen::Vector3d pos_rot2= kin.get_rot_p();
    Eigen::Vector3d pos_ee2= kin.get_ee_p();

    cout << "\nrpy finale:\n" << rpy3;
    cout << "\nrrot finale:\n" << pos_rot2;
    cout << "\nq finale:\n" << Thf;
    cout << "\nee finale:\n" << pos_ee2;


    cout << "\n----------------------------------------------" << endl;
    cout << "\n----------------------------------------------" << endl;
    cout << "\n----------------------------------------------" << endl;

    Eigen::MatrixXd J = kin.compute_J(Th);
    J =  kin.geo2anJ(J, kin.T0e);
    cout << "\nq:\n" << Th;
    cout << "\nJac:\n" << J;

    cout << "\ninv Jac:\n" << J.inverse();


    //vector<Eigen::Vector < double, 6 >> path = kin.compute_ik(pos2,rpy, Th, 100, false);
    

    //kin.compute_fc(path[path.size()-1]);
    
    cout << "\n\tfinal pos: " << kin.get_ee_p();

    bool print_ = true;
*/

     Eigen::Vector3d pos2(0.1, -0.5, -0.7);
    //pos2={0.8, 0.5, -0.7};
    Eigen::Vector3d rpy(0,0,0);
    Eigen::Vector < double, 6 > q = Eigen::Vector < double, 6 > (0.024495132178144807,-0.6710569050151003,-0.32897814282124016,-0.019925843179758207,0.016599836161831028,6.478812432020075e-17);
    
    Eigen::Vector3d ee;

    Eigen::Vector<double, 6> Th1(-2.35578, -3.17425, 0.275255, 6.194, 1.262, 7.29836);
    Eigen::Vector<double, 6> Th2(-1.8774478, -3.37425, 0.475255, 5.794, 1.962, 6.89836);

    kin.compute_fc(Th1);
    rpy=kin.get_rot_p();

    cout << "\nrpy1: " << rpy;

    kin.compute_fc(Th2);
    rpy=kin.get_rot_p();
    ee=kin.get_ee_p();

    cout << "\nrpy2: " << rpy;

    vector<Eigen::Vector < double, 6 >> path = kin.compute_ik(ee,rpy,Th1,50, false);

    bool print_ = false;
    if(print_){
            std::cout << "path: \n";

            for (Eigen::Vector < double, 6 > i: path)
                std::cout << i << "\n\n";

            std::cout << "\n--------------------------------------------\n";
        }

    ofstream myfile;
    myfile.open ("path.txt");

    for(int i=0;i<path.size();i++){
        for(int j=0; j<6;j++){
        myfile << path[i].coeff(j);
        if(j<5)
            myfile << ",";
        
        }
        myfile << endl;
    }
    //myfile << "Writing this to a file.\n";


    cout << "\nsize of pathz: " << path.size();



    cout << "fine " << endl;
}
