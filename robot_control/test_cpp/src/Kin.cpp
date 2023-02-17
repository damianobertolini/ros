#ifndef KIN__CPP
#define KIN__CPP

#include "Eigen/Eigen/Dense"
#include <iostream>
#include <vector>
#include <cmath>
#include "Helper.cpp"
#include <fstream>

//using namespace pinocchio;
using namespace std;


#ifndef MODEL_PATH
  #define MODEL_PATH "~/ros_ws/src/robo/ros/robot_urdf/generated_urdf/ur5.urdf"
#endif

class Kin {
   

    public:

        Eigen::Vector < double, 7 > arm_l; //lunghezza bracci
        //Eigen::Vector<double, 6> joint_q; //rotazione joints
        Eigen::Vector < double, 6 > Th;
        Eigen::Vector < double, 6 > A;
        Eigen::Vector < double, 6 > D;
        Eigen::Vector <double , 6> alfa;
        Eigen::Matrix4d T10m;
        Eigen::Matrix4d T21m;
        Eigen::Matrix4d T32m;
        Eigen::Matrix4d T43m;
        Eigen::Matrix4d T54m;
        Eigen::Matrix4d T65m;
        Eigen::Matrix4d T0e;
        Eigen::Matrix4d T0;
        //Eigen::Matrix4d T10m;
        Eigen::Matrix4d T20m;
        Eigen::Matrix4d T30m;
        Eigen::Matrix4d T40m;
        Eigen::Matrix4d T50m;
        Eigen::Matrix4d T60m;
        Eigen::MatrixXd J;
        static inline int ik_index=7;


        Kin() {
            arm_l << 0.089159, 0.13585, 0.425, 0.1197, 0.39225, 0.094, 0.068;
            Th = Eigen::Vector < double, 6 > (0.089159, 0, 0, 0.10915, 0.09465, 0.0823);
            A = Eigen::Vector <double , 6> (0, -0.425, -0.3922, 0, 0, 0);//distanze su A
            D = Eigen::Vector <double , 6> (0.1625, 0, 0, 0.1333, 0.0997, 0.0996);//dimensione bracci
            alfa = Eigen::Vector <double , 6> (0, M_PI/2, 0, 0, M_PI/2, -M_PI/2);
            J = Eigen::MatrixXd(6,6);
        }
    

    Eigen::Matrix4d get_T0e(){
        return T0e;
    }

    Eigen::Vector3d get_rot_p(){ //get end effector rotation(x,y,z)

        Eigen::Vector3d pos;

        //cout << "eigen T0e coeff 0,3" << T0e.coeff(0,3) << endl;;
        pos << T0e.coeff(0,3), T0e.coeff(1,3), T0e.coeff(2,3);
        return pos;
    }

    Eigen::Vector < double, 6 > get_pr_now(){

        Eigen::Vector < double, 6 > pr_i;

        pr_i(0) = get_ee_p()[0];
        pr_i(1) = get_ee_p()[1];
        pr_i(2) = get_ee_p()[2];
        pr_i(3) = rotm2eul(T0e)[0];
        pr_i(4) = rotm2eul(T0e)[1];
        pr_i(5) = rotm2eul(T0e)[2];//sovrascrivo con la posizione attuale

        return pr_i;
    }


    Eigen::Vector3d get_ee_p(){ //get end effector position(x,y,z)

        Eigen::Vector3d pos;

        //cout << "eigen T0e coeff 0,3" << T0e.coeff(0,3) << endl;;
        pos << T0e.coeff(0,3), T0e.coeff(1,3), T0e.coeff(2,3);
        return pos;
    }

    int  compute_fc(Eigen::Vector < double, 6 > Th, bool print_ = false) {
        this->Th=Th;

        //std::cout << "computing fc\n";
        //print_=true;

        T10m = T10f(Th[0]);
        T21m = T21f(Th[1]);
        T32m = T32f(Th[2]);
        T43m = T43f(Th[3]);
        T54m = T54f(Th[4]);
        T65m = T65f(Th[5]);

        T60m =T10m*T21m*T32m*T43m*T54m*T65m;
        T0e=T60m;



        if(print_){
            std::cout << "T10m:\n " << T10m << std::endl;
            std::cout << "T21m:\n " << T21m << std::endl;
            std::cout << "T32m:\n " << T32m << std::endl;
            std::cout << "T43m:\n " << T43m << std::endl;
            std::cout << "T54m:\n " << T54m << std::endl;
            std::cout << "T65:\n " << T65m << std::endl;
            std::cout << "T0e: " << T0e << endl;
            std::cout << "done computing fc\n";
        }
        return 0;
    }

    Eigen::MatrixXd compute_J(Eigen::Vector < double, 6 > q, bool print_ = false){

        //if(q == NULL)
        //    q = Th;//posso passare senza parametri

        Eigen::Vector < double, 3 > p;
        compute_fc(q);//aggiorno le matrici T con questa q

        //std::cout << "building param\n" ;

        double A1 = A(0); double A2 = A(1); double A3 = A(2); double A4 = A(3); double A5 = A(4);  double A6 = A(5);
        double D1 = D(0); double D2 = D(1); double D3 = D(2); double D4 = D(3); double D5 = D(4);  double D6 = D(5);

        double th1 = Th[0];
        double th2 = Th[1];
        double th3 = Th[2];
        double th4 = Th[3];
        double th5 = Th[4];
        double th6 = Th[5];


        Eigen::Vector < double, 6 > J1;
        Eigen::Vector < double, 6 > J2;
        Eigen::Vector < double, 6 > J3;
        Eigen::Vector < double, 6 > J4;
        Eigen::Vector < double, 6 > J5;
        Eigen::Vector < double, 6 > J6;

        //std::cout << "building jac\n" ;
        //std::cout << "test: \n" << (D5*(cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5)) + D3*cos(th1) + D4*cos(th1) - A3*cos(th2 + th3)*sin(th1) - A2*cos(th2)*sin(th1) - D5*sin(th2 + th3 + th4)*sin(th1)) << "\n";
         J1 <<
            D5*(cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5)) + D3*cos(th1) + D4*cos(th1) - A3*cos(th2 + th3)*sin(th1) - A2*cos(th2)*sin(th1) - D5*sin(th2 + th3 + th4)*sin(th1),
            D5*(cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5)) + D3*sin(th1) + D4*sin(th1) + A3*cos(th2 + th3)*cos(th1) + A2*cos(th1)*cos(th2) + D5*sin(th2 + th3 + th4)*cos(th1),
            0,
            0,
            0,
            1;
        J2 <<
            -cos(th1)*(A3*sin(th2 + th3) + A2*sin(th2) + D5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) - D5*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4))),
            -sin(th1)*(A3*sin(th2 + th3) + A2*sin(th2) + D5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) - D5*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4))),
            A3*cos(th2 + th3) - (D5*sin(th2 + th3 + th4 + th5))/2 + A2*cos(th2) + (D5*sin(th2 + th3 + th4 - th5))/2 + D5*sin(th2 + th3 + th4),
            sin(th1),
            -cos(th1),
            0;
        J3 <<
            cos(th1)*(D5*cos(th2 + th3 + th4) - A3*sin(th2 + th3) + D5*sin(th2 + th3 + th4)*sin(th5)),
            sin(th1)*(D5*cos(th2 + th3 + th4) - A3*sin(th2 + th3) + D5*sin(th2 + th3 + th4)*sin(th5)),
            A3*cos(th2 + th3) - (D5*sin(th2 + th3 + th4 + th5))/2 + (D5*sin(th2 + th3 + th4 - th5))/2 + D5*sin(th2 + th3 + th4),
            sin(th1),
            -cos(th1),
            0;
        J4 <<
            D5*cos(th1)*(cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*sin(th5)),
            D5*sin(th1)*(cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*sin(th5)),
            D5*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4) - sin(th2 + th3 + th4 + th5)/2),
            sin(th1),
            -cos(th1),
            0;
        J5 <<
            -D5*sin(th1)*sin(th5) - D5*cos(th2 + th3 + th4)*cos(th1)*cos(th5),
            D5*cos(th1)*sin(th5) - D5*cos(th2 + th3 + th4)*cos(th5)*sin(th1),
            -D5*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4 + th5)/2),
            sin(th2 + th3 + th4)*cos(th1),
            sin(th2 + th3 + th4)*sin(th1),
            -cos(th2 + th3 + th4);
        J6 <<
            0,
            0,
            0,
            cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5),
            -cos(th1)*cos(th5) - cos(th2 + th3 + th4)*sin(th1)*sin(th5),
            -sin(th2 + th3 + th4)*sin(th5);

        Eigen::MatrixXd tmp(6,6);
        tmp << J1,J2,J3,J4,J5,J6;
        J = tmp;

        if(print_)
            std::cout << "done building jac\n" ;

        return J;
    }

    int eval_ik_index(Eigen::Vector < double, 6 > j_now){
        Helper h;
        
        j_now = h.constrainAngle180(j_now);
        //cout << "joint attuale pre wrap: " << j_now << endl;
        compute_fc(j_now);
        Eigen::Vector < double, 6 > pr_i = get_pr_now();
        Eigen::Vector < double, 6 > tmp;

        //cout << "pose end effector : " << pr_i<< endl;

        double delta=0;
        std::vector<Eigen::Vector < double, 6 >> ik = compute_ik(pr_i);
        //cout << "joint ik pre wrap: \n";
        for(int i=0; i<ik.size();i++){
            //cout << i << ")\n";
            for(int j=0; j< 6; j++){
                //cout << ik[i](j) << ",";
            }
            //cout << "\n";
        }
        Eigen::Vector < double, 8 > delta_vec;
        //cout << "-----------------------------------------";
        for(int i=0; i< ik.size(); i++){
            delta =0;
            ik[i] = h.constrainAngle(ik[i]);//devo wrappare tutti e 2 altrimenti sono sfasati
            for(int j=0; j< 6; j++){
                delta+=h.dist_constrain(std::abs(h.constrainAngle(ik[i](j))-j_now(j)));
                //cout << "\n"  << h.constrainAngle(ik[i](j))  << " - " << j_now(j) << "=" << h.dist_constrain(std::abs(h.constrainAngle(ik[i](j))-j_now(j)));
            }
            //cout << "\n\n";
            delta_vec(i)=delta;
        }
        int index=0;
        double min = delta_vec(0);

        for(int i=0; i< 8; i++){//trovo il minimo e l'index
            if(delta_vec(i)<min){
                min = delta_vec(i);
                index = i;
            }
        }

        Kin::ik_index = index;

        //cout << "joint attuale: " << j_now << endl;

        //cout << "joint ik: " << ik[index] << endl;
        
        cout << "errore minimo: " << min << "  a i: " << index <<  endl;

        //cout << "tutti gli errori: " << delta_vec << endl;

        //cout << "tutte le config: " << endl;
        /*
        for(int i=0; i<ik.size();i++){
            cout << i << ")\n";
            for(int j=0; j< 6; j++){
                cout << ik[i](j) << ",";
            }
            cout << "\n";
        }
        */
        return ik_index;
    }

    std::vector<Eigen::Vector < double, 6 >> da_a(std::vector<Eigen::Vector < double, 6 >> path_pr,Eigen::Vector < double, 6 > th0, Eigen::MatrixXd k, int steps){
        
        Eigen::Vector < double, 6 > q = th0;
        std::vector<Eigen::Vector < double, 6 >> path;
        path.push_back(q);
        Eigen::Vector < double, 6 > qkdot;
        Eigen::Vector < double, 6 > qk1;
        Eigen::Vector < double, 6 > tmp;
        Eigen::Vector3d p_now;
        Eigen::Vector3d rpy_now;
        float max_qdot=0;
 
        Helper help;
        ofstream myfile;
        myfile.open ("path_p_real.txt");

        for(int i=1; i< steps; i++){//parto da 1

            compute_fc(q);//carico le matrici
            p_now=get_ee_p();
            rpy_now=rotm2eul(T0e);
            Eigen::Vector < double, 6 > pr_now;
            Eigen::Vector < double, 6 > delta;
        
            pr_now << p_now,rpy_now;
            myfile << pr_now(0)<< "," <<pr_now(1)<< "," <<pr_now(2)<< "," <<pr_now(3)<< "," <<pr_now(4)<< "," <<pr_now(5)<< "\n";
            

            delta = path_pr[i]-path_pr[i-1];//parto da 1 perché la q0 la ho gia
            qkdot = invDiffKinematiControlComplete2(q,pr_now,delta,path_pr[i],k);
            
            q=q + qkdot;
            path.push_back(q);

            if(false){
                string s;
                
                cout << "\n\nq: " << q <<"\n\n pr_now: " << pr_now << "\n\n delta: " << delta << "\n\n path_pr[i]: " << path_pr[i] << endl << endl;
                cout << i <<" ) qkdot: " << qkdot << endl;
                cout << "---------------------------------------------" << endl;
                //cin >> s;
            }
            if(help.abs(qkdot)>max_qdot)
                max_qdot=help.abs(qkdot);
            myfile.close();

        }

        cout << "max qdot: " << max_qdot << endl;

        return path;
    }

    Eigen::Vector < double, 6 > invDiffKinematiControlComplete2(Eigen::Vector < double, 6 > q_temp,  Eigen::Vector < double, 6 > pr_now, Eigen::Vector < double, 6 > delta, Eigen::Vector < double, 6 > path_pr_i,    Eigen::MatrixXd k){

        Eigen::Vector < double, 6 > dotqk;
        compute_J(q_temp);
        float alpha, beta, gamma;
        alpha=pr_now(3); //z
        beta=pr_now(4);    //y
        gamma=pr_now(5);   //x

        Eigen::Matrix3d Tcc;
        Eigen::MatrixXd Tcca = Eigen::MatrixXd(6,6);
        Eigen::MatrixXd Jcca = Eigen::MatrixXd(6,6);
        Eigen::Vector < double, 6 > tmp; //stack do vel e rot modulati
        Tcc << cos(beta)*cos(gamma), -sin(gamma), 0,
            cos(beta)*sin(gamma), cos(gamma), 0,
            -sin(beta), 0, 1;

        Tcca << 1,	0,	0,	0,	0,	0,
                0,	1,	0,	0,	0,	0,
                0,	0,	1,	0,	0,	0,
                0,	0,	0,	Tcc(0,0),	Tcc(0,1),	Tcc(0,2),
                0,	0,	0,	Tcc(1,0),	Tcc(1,1),	Tcc(1,2),
                0,	0,	0,	Tcc(2,0),	Tcc(2,1),	Tcc(2,2);
        Jcca<< Tcca.inverse()*J;

        dotqk = Jcca.inverse()*( delta + k*(path_pr_i-pr_now)   );

        return dotqk;

    }

    std::vector<Eigen::Vector < double, 6 >> p2p(Eigen::Vector < double, 6 > pr_i, Eigen::Vector < double, 6 > pr_f, int steps = 3000, double minT = 0){
        
        Eigen::Vector < double, 6 > q_i = compute_ik(pr_i)[Kin::ik_index];
        Eigen::Vector < double, 6 > q_f = compute_ik(pr_f)[Kin::ik_index];

        cout << "\n\t(kin p2p) da joints: " << q_i;
        cout << "\n\t(kin p2p) a  joints: " << q_f << endl;;

        Eigen::Matrix4d M;
        std::vector<Eigen::Vector < double, 6 >> path;
        //double minT = 0;
        double maxT = ((double)steps)/1000;
        //path.push_back(q_i);
        Eigen::Vector < double, 6 > qkdot;
        Eigen::Vector < double, 6 > qk1;
        Eigen::Vector < double, 6 > tmp;
        std::vector< Eigen::Vector4d > A;
        Eigen::Vector3d p_now;
        Eigen::Vector3d rpy_now;
        Eigen::Vector4d b;
        Eigen::Vector4d a;

        //cout << "i: \n" <<  q_i <<  endl ;
        //cout << "f: \n" <<  q_f <<  endl ;

        M << 1, minT, pow(minT,2), pow(minT,3),
                0 , 1   , 2*minT,    3*pow(minT,2),
                1, maxT, pow(maxT,2), pow(maxT,3),
                0 , 1   ,2*maxT,    3*pow(maxT,2);

        //std::cout << "maxT:" << maxT << std::endl;
        //std::cout << "M:" << M << std::endl;

        for(int i=0; i<6;i++){
            b << q_i(i) , 0 , q_f(i), 0;
            a = M.inverse()*b;

            //std::cout << "b:" << b << std::endl;
            //std::cout << "i: " << i  << "   num: " << a << std::endl;

            A.push_back(a);
        } 
        /*
        cout << "\n";
        for(int i=0; i<6;i++){
            std::cout << "test2:" << A[i] << std::endl;
        } 
        */
        //cout << "\n";

        tmp << 0,0,0,0,0,0;

        

        for(int i=0;i<steps;i++){
            
            double dt=((double)i)/1000;

            for(int j=0;j<6;j++){
                double q = A[j](0) + A[j](1)*dt + A[j](2)*dt*dt + A[j](3)*dt*dt*dt;
                tmp(j)=q;
                //cout << "i: " << i << "  j: " << j << std::endl;
                //std::cout << "A[j](1)*dt:" << A[j](1)*dt << std::endl;
                //std::cout << "A[j](2)*dt*dt:" << A[j](2)*dt*dt << std::endl;
                //std::cout << "A[j](3)*dt*dt*dt:" << A[j](3)*dt*dt*dt << std::endl;
            } 
            //cout << "\n";
            path.push_back(tmp);
            //cout << "\npushed: " << tmp;
        } 
        

        return path;
    }

    vector<Eigen::Vector < double, 6 >> compute_ik(Eigen::Vector < double, 6 > pr_f){
        Eigen::Vector < double, 6 > th_ik;
        Eigen::Vector4d p50;
        Eigen::Matrix < double, 4, 1 > col4;
        Eigen::Vector4d v4;
        Eigen::Vector3d rpy;
        Eigen::Vector3d p60;
        p60 << pr_f(0),pr_f(1),pr_f(2);
        rpy << pr_f(3),pr_f(4),pr_f(5);
        Eigen::Matrix4d T60;
        Eigen::Matrix3d rot = eul2rotZYX(rpy);

        for(int i=0;i<3;i++){
            for(int k=0;k<3;k++){
                T60(i,k)=rot(i,k);
            }
        }
        T60(0,3)=pr_f(0);
        T60(1,3)=pr_f(1);
        T60(2,3)=pr_f(2);
        T60(3,0)=0;
        T60(3,1)=0;
        T60(3,2)=0;
        T60(3,3)=1;

        v4 << 0,0,0-D(5),1;
        p50 = T60*v4;

        double th1_1 = real(atan2(p50(1), p50(0)) + safe_acos(D(3)/hypot(p50(1), p50(0))))+M_PI/2;
        double th1_2 = real(atan2(p50(1), p50(0)) - safe_acos(D(3)/hypot(p50(1), p50(0))))+M_PI/2;

        double th5_1 = +safe_acos((p60(0)*sin(th1_1) - p60(1)*cos(th1_1)-D(3)) / D(5));
        double th5_2 = -safe_acos((p60(0)*sin(th1_1) - p60(1)*cos(th1_1)-D(3)) / D(5));
        double th5_3 = +safe_acos((p60(0)*sin(th1_2) - p60(1)*cos(th1_2)-D(3)) / D(5));
        double th5_4 = -safe_acos((p60(0)*sin(th1_2) - p60(1)*cos(th1_2)-D(3)) / D(5));
        
        Eigen::Matrix4d T06 = T60.inverse();

        Eigen::Vector3d Xhat = get_col(T06,0);
        Eigen::Vector3d Yhat = get_col(T06,1);

        double th6_1 = real(atan2(((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1)))/sin(th5_1), ((Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1)))/sin(th5_1)));
        double th6_2 = real(atan2(((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1))/sin(th5_2)), ((Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1))/sin(th5_2))));
        double th6_3 = real(atan2(((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2))/sin(th5_3)), ((Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2))/sin(th5_3))));
        double th6_4 = real(atan2(((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2))/sin(th5_4)), ((Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2))/sin(th5_4))));

        Eigen::Matrix4d T41m = T10f(th1_1).inverse()   *  T60  *   T65f(th6_1).inverse() *  T54f(th5_1).inverse();
        Eigen::Vector3d p41_1 = get_col(T41m,3);
        double p41xz_1 = hypot(p41_1(0), p41_1(2));

        T41m = T10f(th1_1).inverse()   *  T60  *   T65f(th6_2).inverse() *  T54f(th5_2).inverse();
        Eigen::Vector3d p41_2 = get_col(T41m,3);
        double p41xz_2 = hypot(p41_2(0), p41_2(2));

        T41m = T10f(th1_2).inverse()   *  T60  *   T65f(th6_3).inverse() *  T54f(th5_3).inverse();
        Eigen::Vector3d p41_3 = get_col(T41m,3);
        double p41xz_3 = hypot(p41_3(0), p41_3(2));

        T41m = T10f(th1_2).inverse()   *  T60  *   T65f(th6_4).inverse() *  T54f(th5_4).inverse();
        Eigen::Vector3d p41_4 = get_col(T41m,3);
        double p41xz_4 = hypot(p41_4(0), p41_4(2));

        double th3_1 = safe_acos((pow(p41xz_1,2)-pow(A(1),2)-pow(A(2),2))/(2*A(1)*A(2)));
        double th3_2 = safe_acos((pow(p41xz_2,2)-pow(A(1),2)-pow(A(2),2))/(2*A(1)*A(2)));
        double th3_3 = safe_acos((pow(p41xz_3,2)-pow(A(1),2)-pow(A(2),2))/(2*A(1)*A(2)));
        double th3_4 = safe_acos((pow(p41xz_4,2)-pow(A(1),2)-pow(A(2),2))/(2*A(1)*A(2)));
        /*
        cout << "pr_f:" << pr_f;

        cout << "th3_1 : " << th3_1 << endl;
        cout << "p41xz_1 " << p41xz_1 <<  endl << endl;
        cout << "th3_2 : " << th3_2 << endl;
        cout << "p41xz_2 " << p41xz_2 <<  endl<< endl;
        cout << "th3_3 : " << th3_3 << endl;
        cout << "p41xz_3 " << p41xz_3 <<  endl<< endl;
        cout << "th3_4 : " << th3_4 << endl;
        cout << "p41xz_4 " << p41xz_4 <<  endl<< endl;
        */
        double th3_5 = -th3_1;
        double th3_6 = -th3_2;
        double th3_7 = -th3_3;
        double th3_8 = -th3_4;
        
        /*
        double th2_1 = real(atan2(-p41_1(2), -p41_1(0))-asin((-A(2)*sin(th3_1))/p41xz_1));
        double th2_2 = real(atan2(-p41_2(2), -p41_2(0))-asin((-A(2)*sin(th3_2))/p41xz_2));
        double th2_3 = real(atan2(-p41_3(2), -p41_3(0))-asin((-A(2)*sin(th3_3))/p41xz_3));
        double th2_4 = real(atan2(-p41_4(2), -p41_4(0))-asin((-A(2)*sin(th3_4))/p41xz_4));

        double th2_5 = real(atan2(-p41_1(2), -p41_1(0))-asin((A(2)*sin(th3_1))/p41xz_1));
        double th2_6 = real(atan2(-p41_2(2), -p41_2(0))-asin((A(2)*sin(th3_2))/p41xz_2));
        double th2_7 = real(atan2(-p41_3(2), -p41_3(0))-asin((A(2)*sin(th3_3))/p41xz_3));
        double th2_8 = real(atan2(-p41_4(2), -p41_4(0))-asin((A(2)*sin(th3_4))/p41xz_4));
        */

        double th2_1 = real(atan2(-p41_1(2), -p41_1(0))-safe_asin((-A(2)*sin(th3_1))/p41xz_1));
        double th2_2 = real(atan2(-p41_2(2), -p41_2(0))-safe_asin((-A(2)*sin(th3_2))/p41xz_2));
        double th2_3 = real(atan2(-p41_3(2), -p41_3(0))-safe_asin((-A(2)*sin(th3_3))/p41xz_3));
        double th2_4 = real(atan2(-p41_4(2), -p41_4(0))-safe_asin((-A(2)*sin(th3_4))/p41xz_4));
        
        double th2_5 = real(atan2(-p41_1(2), -p41_1(0))-safe_asin((A(2)*sin(th3_1))/p41xz_1));
        double th2_6 = real(atan2(-p41_2(2), -p41_2(0))-safe_asin((A(2)*sin(th3_2))/p41xz_2));
        double th2_7 = real(atan2(-p41_3(2), -p41_3(0))-safe_asin((A(2)*sin(th3_3))/p41xz_3));
        double th2_8 = real(atan2(-p41_4(2), -p41_4(0))-safe_asin((A(2)*sin(th3_4))/p41xz_4));

        Eigen::Matrix4d T43m = (T32f(th3_1)).inverse()*(T21f(th2_1)).inverse()*(T10f(th1_1)).inverse()*T60*(T65f(th6_1)).inverse()*(T54f(th5_1)).inverse();
        Eigen::Vector3d Xhat43 = get_col(T43m,0);
        double th4_1 = real(atan2(Xhat43(1), Xhat43(0)));

                        T43m = (T32f(th3_2)).inverse()*(T21f(th2_2)).inverse()*(T10f(th1_1)).inverse()*T60*(T65f(th6_2)).inverse()*(T54f(th5_2)).inverse();
                        Xhat43 = get_col(T43m,0);
        double th4_2 = real(atan2(Xhat43(1), Xhat43(0)));

                        T43m = (T32f(th3_3)).inverse()*(T21f(th2_3)).inverse()*(T10f(th1_2)).inverse()*T60*(T65f(th6_3)).inverse()*(T54f(th5_3)).inverse();
                        Xhat43 = get_col(T43m,0);
        double th4_3 = real(atan2(Xhat43(1), Xhat43(0)));

                        T43m = (T32f(th3_4)).inverse()*(T21f(th2_4)).inverse()*(T10f(th1_2)).inverse()*T60*(T65f(th6_4)).inverse()*(T54f(th5_4)).inverse();
                        Xhat43 = get_col(T43m,0);
        double th4_4 = real(atan2(Xhat43(1), Xhat43(0)));

                        T43m = (T32f(th3_5)).inverse()*(T21f(th2_5)).inverse()*(T10f(th1_1)).inverse()*T60*(T65f(th6_1)).inverse()*(T54f(th5_1)).inverse();
                        Xhat43 = get_col(T43m,0);
        double th4_5 = real(atan2(Xhat43(1), Xhat43(0)));

                        T43m = (T32f(th3_6)).inverse()*(T21f(th2_6)).inverse()*(T10f(th1_1)).inverse()*T60*(T65f(th6_2)).inverse()*(T54f(th5_2)).inverse();
                        Xhat43 = get_col(T43m,0);
        double th4_6 = real(atan2(Xhat43(1), Xhat43(0)));

                        T43m = (T32f(th3_7)).inverse()*(T21f(th2_7)).inverse()*(T10f(th1_2)).inverse()*T60*(T65f(th6_3)).inverse()*(T54f(th5_3)).inverse();
                        Xhat43 = get_col(T43m,0);
        double th4_7 = real(atan2(Xhat43(1), Xhat43(0)));

                        T43m = (T32f(th3_8)).inverse()*(T21f(th2_8)).inverse()*(T10f(th1_2)).inverse()*T60*(T65f(th6_4)).inverse()*(T54f(th5_4)).inverse();
                        Xhat43 = get_col(T43m,0);
        double th4_8 = real(atan2(Xhat43(1), Xhat43(0)));

        Eigen::Matrix<double, 8,6> Th_inverse;
        
        
        Eigen::Vector < double, 6 > a1;
        Eigen::Vector < double, 6 > a2;
        Eigen::Vector < double, 6 > a3;
        Eigen::Vector < double, 6 > a4;
        Eigen::Vector < double, 6 > a5;
        Eigen::Vector < double, 6 > a6;
        Eigen::Vector < double, 6 > a7;
        Eigen::Vector < double, 6 > a8;

        a1  <<  th1_1,th2_1,th3_1,th4_1,th5_1,th6_1;
        a2  <<  th1_1,th2_2,th3_2,th4_2,th5_2,th6_2;
        a3  <<  th1_2,th2_3,th3_3,th4_3,th5_3,th6_3;
        a4  <<  th1_2,th2_4,th3_4,th4_4,th5_4,th6_4;
        a5  <<  th1_1,th2_5,th3_5,th4_5,th5_1,th6_1;
        a6  <<  th1_1,th2_6,th3_6,th4_6,th5_2,th6_2;
        a7  <<  th1_2,th2_7,th3_7,th4_7,th5_3,th6_3;
        a8  <<  th1_2,th2_8,th3_8,th4_8,th5_4,th6_4;


        vector<Eigen::Vector < double, 6 >> vac_th_inverse;
        vac_th_inverse.push_back(a1);
        vac_th_inverse.push_back(a2);
        vac_th_inverse.push_back(a3);
        vac_th_inverse.push_back(a4);
        vac_th_inverse.push_back(a5);
        vac_th_inverse.push_back(a6);
        vac_th_inverse.push_back(a7);
        vac_th_inverse.push_back(a8);

        return vac_th_inverse;
    }

    Eigen::Vector3d get_col(Eigen::Matrix4d M, int i){
        Eigen::Vector3d ret;
        ret<< M(0,i),M(1,i),M(2,i);
        return ret;
    }

    double hypot(double a, double b){
        return sqrt(pow(a,2)+pow(b,2));
    }

    Eigen::Vector3d pr_to_r(Eigen::Vector < double, 6 > pr){
        Eigen::Vector3d r;
        r<< pr(3),pr(4),pr(5);
        return r;
    }
    
    Eigen::Vector3d pr_to_p(Eigen::Vector < double, 6 > pr){
        Eigen::Vector3d p;
        p<< pr(0),pr(1),pr(2);
        return p;
    }

    Eigen::Matrix3d eul2rotXYZ(Eigen::Vector3d rpy){
        double c_roll =  cos(rpy[0]);
        double s_roll = sin(rpy[0]);
        double c_pitch =   cos(rpy[1]) ;      
        double s_pitch = sin(rpy[1]);
        double c_yaw = cos(rpy[2]);
        double s_yaw = sin(rpy[2]);
        Eigen::Matrix3d Rx,Ry,Rz,R;
        Eigen::Matrix4d rot;
        Rx <<   1   ,         0           ,        0, 
            0   ,        c_roll  ,  -s_roll,
            0   ,      s_roll,      c_roll;

        Ry << c_pitch     ,     0  ,   s_pitch,
                0       ,    1  ,   0 ,
                -s_pitch     ,    0   ,  c_pitch;
                

        Rz <<  c_yaw  ,  -s_yaw ,        0 ,
                s_yaw ,  c_yaw ,          0 ,
                0      ,     0     ,       1 ;

        return (Rx*(Ry*Rz)); //xyz
    }

    Eigen::Matrix3d eul2rotZYX(Eigen::Vector3d rpy){
        double c_roll =  cos(rpy[0]);
        double s_roll = sin(rpy[0]);
        double c_pitch =   cos(rpy[1]) ;      
        double s_pitch = sin(rpy[1]);
        double c_yaw = cos(rpy[2]);
        double s_yaw = sin(rpy[2]);
        Eigen::Matrix3d Rx,Ry,Rz,R;
        Eigen::Matrix4d rot;
        Rx <<   1   ,         0           ,        0, 
            0   ,        c_roll  ,  -s_roll,
            0   ,      s_roll,      c_roll;

        Ry << c_pitch     ,     0  ,   s_pitch,
                0       ,    1  ,   0 ,
                -s_pitch     ,    0   ,  c_pitch;
                

        Rz <<  c_yaw  ,  -s_yaw ,        0 ,
                s_yaw ,  c_yaw ,          0 ,
                0      ,     0     ,       1 ;

        R = (Rz*(Ry*Rx));

        swap(R(0,0),R(2,2));
        swap(R(0,1),R(1,2));
        swap(R(1,0),R(2,1)); //per zyx

        return R;
    }

    void swap(double& f1,double& f2){
        double buff;
        buff=f1;
        f1=f2;
        f2=buff;
    }

    std::vector<Eigen::Vector < double, 6 >> fillpath(Eigen::Vector < double, 6 > q, Eigen::Vector < double, 6 > pr_f, int steps){
        compute_fc(q);//carico le matrici

        std::vector<Eigen::Vector < double, 6 >> path;
        Eigen::Vector3d p_i;
        Eigen::Vector3d rpy_i;
        Eigen::Vector < double, 6 > delta;
        Eigen::Vector < double, 6 > pr_i;

        //cout << " pushing  q \n";

        //path.push_back(q);

        //cout << " pushed q:" << q<<endl<< endl;

        p_i = get_ee_p();
        rpy_i=rotm2eul(T0e);

        pr_i << p_i,rpy_i;//posizione i

        path.push_back(pr_i);//pr iniziale

        delta = (pr_f-pr_i)/steps;

        for(int i=1; i< steps;i++){
            pr_i = pr_i+delta;
            path.push_back(pr_i);
            //cout << " pushed pr:" << pr << endl;
        }
        


        return path;
    }
    
    Eigen::Vector < double, 6 > invDiffKinematiControlComplete(Eigen::Vector < double, 6 > q_temp,  Eigen::Vector3d p_now, Eigen::Vector3d rpy_now, Eigen::Vector3d p_f, Eigen::Vector3d pry_f,  Eigen::Vector3d vel,    Eigen::Vector3d rot,    Eigen::Matrix3d k,  Eigen::Matrix3d kphi){
        
        Eigen::Vector < double, 6 > dotqk;

        compute_J(q_temp);
        float alpha, beta, gamma;
        alpha=rot(2); //z
        beta=rot(1);    //y
        gamma=rot(0);   //x

        Eigen::Matrix3d Tcc;
        Eigen::MatrixXd Tcca = Eigen::MatrixXd(6,6);
        Eigen::MatrixXd Jcca = Eigen::MatrixXd(6,6);
        Eigen::Vector < double, 6 > tmp; //stack do vel e rot modulati
        Tcc << cos(beta)*cos(gamma), -sin(gamma), 0,
            cos(beta)*sin(gamma), cos(gamma), 0,
            -sin(beta), 0, 1;

        Tcca << 1,	0,	0,	0,	0,	0,
                0,	1,	0,	0,	0,	0,
                0,	0,	1,	0,	0,	0,
                0,	0,	0,	Tcc(0,0),	Tcc(0,1),	Tcc(0,2),
                0,	0,	0,	Tcc(1,0),	Tcc(1,1),	Tcc(1,2),
                0,	0,	0,	Tcc(2,0),	Tcc(2,1),	Tcc(2,2);
        Jcca<< Tcca.inverse()*J;

        tmp << vel+k*(vel), rot+kphi*rot;

        dotqk = Jcca.inverse()*tmp;

        return dotqk;
    }

    Eigen::Vector3d lin_vel(Eigen::Vector3d p_i, Eigen::Vector3d p_f, int steps = 5000){//5 secondi default
        return (p_f - p_i) / steps;
    }

    Eigen::Vector3d lin_rot(Eigen::Vector3d rpy_i, Eigen::Vector3d rpy_f, int steps = 5000){//5 secondi default
        return (rpy_f - rpy_i) / steps;
    }

    Eigen::Vector3d rotm2eul(Eigen::Matrix4d R){//da matrice a rpy zyx
        Eigen::Vector3d eul;
        /*
        double phi = atan2(R.coeff(1,0), R.coeff(0,0));
        double theta = atan2(-R.coeff(2,0), sqrt(pow(R.coeff(2,1),2) + pow(R.coeff(2,2),2) ));
        double psi = atan2(R.coeff(2,1), R.coeff(2,2));
        */
        double psi = atan2(R.coeff(1,0), R.coeff(0,0));
        double theta = atan2(-R.coeff(2,0), sqrt(pow(R.coeff(2,1),2) + pow(R.coeff(2,2),2) ));
        double phi = atan2(R.coeff(2,1), R.coeff(2,2));
        //unit test should return roll = 0.5 pitch = 0.2  yaw = 0.3
        //rot2eul(np.array([ [0.9363,   -0.1684,    0.3082], [0.2896 ,   0.8665  , -0.4065], [-0.1987 ,   0.4699  ,  0.8601]]))    
        
        //returns roll = psi, pitch = theta,  yaw = phi
        eul << psi, theta, phi;
        return eul;

    }

    int test_tot2eul(){

        Eigen::Matrix4d tmp;
                tmp <<  0.9363,   -0.1684,    0.3082, 0,
                        0.2896 ,   0.8665  , -0.4065, 0,
                        -0.1987 ,   0.4699  ,  0.8601, 0,
                        0, 0 , 0 , 1;
        Eigen::Vector3d eul = rotm2eul(tmp);

        std::cout << "test tor2eul: \n" << eul << std::endl;

        return 0;
    }

    int isnan (double f) {
        return (f != f); 
    }

    double r_nan(double& d){
        if(d!=d)
            return 0;
        return d;
    }


    double safe_acos(const double& value) {
        if (value<=-1) {
            return M_PI;
        } else if (value>=1) {
            return 0;
        } else {
            return acos(value);
        }
    }

    double safe_asin(const double& value) {
        if (value<=-1) {
            return M_PI/2;
        } else if (value>=1) {
            return -M_PI/2;
        } else {
            return asin(value);
        }
    }

    Eigen::MatrixXd geo2anJ(Eigen::MatrixXd J, Eigen::MatrixXd T0e_){

        //std::cout << "geo2ana: \n" << std::endl;

        //std::cout << "col removed: \n" << std::endl;

        Eigen::Vector3d rpy_ee= rotm2eul(T0e_);

        //removeColumn(rpy_ee,3);
        //removeRow(rmpy_ee, 3);//tengo la parte relatva alle rotazioni

        //std::cout << "col removed: \n" << std::endl;

        double roll = rpy_ee[0];
        double pitch = rpy_ee[1];
        double yaw = rpy_ee[2];

        Eigen::Matrix3d T_w;
        T_w <<  cos(yaw)*cos(pitch),  -sin(yaw), 0,
                sin(yaw)*cos(pitch),   cos(yaw), 0,
                -sin(pitch),               0, 1;//conversione omega a euler
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Z = Eigen::Matrix3d::Zero();

        Eigen::MatrixXd tra(6,6);
        Eigen::MatrixXd tmp1(3,6);
        Eigen::MatrixXd tmp2(3,6);

        tmp1 << I,Z;
        tmp2 << Z,T_w;

        //std::cout << "concateno: \n" << std::endl;

        tra << tmp1,tmp2;

        //std::cout << "prodotto: \n" << tra << "\n";
        //std::cout << "righe: \n" << J.rows() << "colonne:\n"<< J.cols() <<  "\n";


        return tra*J;

        
    }

    void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove){
        unsigned int numRows = matrix.rows()-1;
        unsigned int numCols = matrix.cols();

        if( rowToRemove < numRows )
            matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

        matrix.conservativeResize(numRows,numCols);
    }
    int wrap(Eigen::Vector3d& v){
        for(int i=0; i<3;i++){
            if (v(i) > M_PI) 
                v(i) = (2*M_PI - v(i));
        }       
        return 0;    
    }
    

    private:
        Eigen::Matrix4d T10f(double th1) {
            Eigen::Matrix4d tmp;

            tmp <<  cos(th1), -sin(th1), 0, 0,
                    sin(th1), cos(th1), 0, 0,
                    0, 0, 1, D(0),
                    0, 0, 0, 1;
            return tmp;
            //rotazione e traslazione
        }
        Eigen::Matrix4d T21f(double th2) {
            Eigen::Matrix4d tmp;
            tmp <<  cos(th2), -sin(th2), 0, 0,
                    0, 0, -1, 0,
                    sin(th2), cos(th2), 0, 0,
                    0, 0, 0, 1;
            return tmp;
        }
        Eigen::Matrix4d T32f(double th3) {

            Eigen::Matrix4d tmp;
            tmp <<  cos(th3), -sin(th3), 0, A(1),
                    sin(th3), cos(th3), 0, 0,
                    0, 0, 1, D(2),
                    0, 0, 0, 1;

            return tmp;
        }
        Eigen::Matrix4d T43f(double th4) {
            Eigen::Matrix4d tmp;
            tmp << cos(th4), -sin(th4), 0, A(2),
                    sin(th4), cos(th4), 0, 0,
                    0, 0, 1, D(3),
                    0, 0, 0, 1;
            

            return tmp;
        }
        Eigen::Matrix4d T54f(double th5) {
            Eigen::Matrix4d tmp;
            tmp << cos(th5), -sin(th5), 0, 0,
                    0, 0, -1, -D(4),
                    sin(th5), cos(th5), 0, 0,
                    0, 0, 0, 1;

            return tmp; 
        }
        Eigen::Matrix4d T65f(double th6) {
            Eigen::Matrix4d tmp;
            tmp << cos(th6), -sin(th6), 0, 0,
                    0, 0, 1, D(5),
                    -sin(th6), -cos(th6), 0, 0,
                    0, 0, 0, 1;

            return tmp; 
        }
};


#endif