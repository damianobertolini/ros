
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>

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

    std::vector<Eigen::Vector < double, 6 >> compute_ik(Eigen::Vector3d p_f,Eigen::Vector3d rpy_f, Eigen::Vector < double, 6 > th0, const int steps=5000, bool print_ = false){
                                        //posizione finale         orientazione finale                    posizione attuale          passi

        //base_controllers/utils/kin_dyn_utils.py
        //base_controllers/utils/math_tools.py

        //trovo wrist pos

       //calcolo le velocità
       //uso velocità lineare
        //compute_fc(th0);//carico vettori rotazione
        Th = th0;
        //Eigen::Vector3d rpy_f;

        std::cout << "computing dik \n" ;

        compute_fc(th0);

        Eigen::Vector3d p_i = get_ee_p();
        Eigen::Vector3d vel = lin_vel(p_i,p_f, steps);

        if(print_)
            std::cout << "vel:" << vel << "\n";

        compute_fc(th0);//carico le matrici con la posizione iniziale

        Eigen::Vector3d rot = lin_rot(rotm2eul(T0e),rpy_f, steps);
        Eigen::Vector < double, 6 > q_temp = Th;

        Eigen::Vector < double, 6 > qkdot;
        Eigen::Vector < double, 6 > qk1;
        Eigen::Vector < double, 6 > tmp;

        std::cout << "builing path \n" ;

        std::vector<Eigen::Vector < double, 6 >> path; //vettore di posizioni

        compute_J(q_temp);//carica anche matrici di rotazione

        Eigen::Vector3d p_now;
        Eigen::Vector3d rpy_now;
        Eigen::Matrix3d k;
        k<< 10,0,0,
            0,10,0,
            0,0,10;
        Eigen::Matrix3d kphi;
        kphi<< 0.01,0,0,
            0,0.01,0,
            0,0,0.01;
        //matrice damping
         std::cout << "starting loop \n" ;
        for(int i =0; i < steps; i++){
            J = compute_J(q_temp); 
            //J =  geo2anJ(J, T0e);
            //devo cambiare jacobiano d ageometrico a analitico
            //Eigen::Matrix<double, 6, 4> J_inv = J.inverse();

            //vel << 0.02,0.03,0.04;
            rot << 0,0,0;
            compute_fc(q_temp);
            p_now = get_ee_p();
            rpy_now=rotm2eul(T0e);

            tmp << vel, rot;//ve, stack di velocita' ee e rotazione
            //std::cout << "inverto\n";
            qkdot = J.inverse()*tmp;

            //std::cout << "before idkcc \n" ;
            qkdot = invDiffKinematiControlComplete(q_temp,p_now,rpy_now,p_f,rpy_f,vel,rot,k,kphi);
            //std::cout << "after idkcc \n" ;

            //std::cout << "invertito\n";
            qk1 = q_temp + qkdot;//1ms
            path.push_back(qk1);
            q_temp = qk1;

            
            if(print_){
                std::cout << "J: \n" << J << "\n";
                //std::cout << "J inv: \n" << J.inverse() << "\n";
                std::cout << "ve: \n" << tmp << "\n";
                std::cout << "qkdot: \n" << qkdot << "\n";
                std::cout << "qk1: \n" << qk1 << "\n";                

            }
            

        }
/*
        if(print_){
            std::cout << "path: \n";

            for (Eigen::Vector < double, 6 > i: path)
                std::cout << i << "\n\n";

            std::cout << "\n--------------------------------------------\n";

            std::cout << "\nfinal pos: \n" << q_temp;
        }

        */
       
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

    Eigen::Vector3d rotm2eul(Eigen::Matrix4d R){//da matrice a rpy
        Eigen::Vector3d eul;

        double phi = atan2(R.coeff(1,0), R.coeff(0,0));
        double theta = atan2(-R.coeff(2,0), sqrt(pow(R.coeff(2,1),2) + pow(R.coeff(2,2),2) ));
        double psi = atan2(R.coeff(2,1), R.coeff(2,2));
       
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
/*
    void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove){
        unsigned int numRows = matrix.rows();
        unsigned int numCols = matrix.cols()-1;

        if( colToRemove < numCols )
            matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

        matrix.conservativeResize(numRows,numCols);
    }
*/

    /*
    int test_pinocchio() {
        pinocchio::Model model;
        pinocchio::buildModels::manipulator(model);
        pinocchio::Data data(model);

        Eigen::VectorXd q = pinocchio::neutral(model);
        Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
        Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

        const Eigen::VectorXd & tau = pinocchio::rnea(model, data, q, v, a);
        std::cout << "tau = " << tau.transpose() << std::endl;

        return 0;
    }
    
    int test_pinocchio2() {

        // You should change here to set up your own URDF file or just pass it as an argument of this example.
        //const std::string urdf_filename = (argc<=1) ? PINOCCHIO_MODEL_DIR + std::string("/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf") : argv[1];
        const std::string urdf_filename = std::string("~/ros_ws/src/robo/ros/robot_urdf/generated_urdf/ur5.urdf");

        // Load the urdf model
        Model model;
        //pinocchio::urdf::buildModel(urdf_filename,model);
        
        pinocchio::buildModels::manipulator(model);
        std::cout << "model name: " << model.name << std::endl;
        // Create data required by the algorithms
        Data data(model);
        
        // Sample a random configuration
        Eigen::VectorXd q = randomConfiguration(model);
        std::cout << "q: " << q.transpose() << std::endl;

        // Perform the forward kinematics over the kinematic tree
        forwardKinematics(model,data,q);

        // Print out the placement of each joint of the kinematic tree
        for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id){
            std::cout << std::setw(24) << std::left
                    << model.names[joint_id] << ": "
                    << std::fixed << std::setprecision(2)
                    << data.oMi[joint_id].translation().transpose()
                    << std::endl;
        }

        return 0;

    }
    int test_pinocchio3() {
        
            pinocchio::Model model;
            pinocchio::buildModels::manipulator(model);
            pinocchio::Data data(model);

            const int JOINT_ID = 6;
            const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1., 0., 1.));

            Eigen::VectorXd q = pinocchio::neutral(model);
            //std::vector<double> a= { 0.02699, -0.66813, -0.32761, -0.0235,   0.02016,  0.0};
            //q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(a.data(), a.size());


            const double eps = 1e-4;
            const int IT_MAX = 1000;
            const double DT = 1e-1;
            const double damp = 1e-6;

            pinocchio::Data::Matrix6x J(6, model.nv);
            J.setZero();

            bool success = false;
            typedef Eigen::Matrix < double, 6, 1 > Vector6d;
            Vector6d err;
            Eigen::VectorXd v(model.nv);
            for (int i = 0;; i++) {
                pinocchio::forwardKinematics(model, data, q);
                const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);
                err = pinocchio::log6(dMi).toVector();
                if (err.norm() < eps) {
                    success = true;
                    break;
                }
                if (i >= IT_MAX) {
                    success = false;
                    break;
                }
                pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J);
                pinocchio::Data::Matrix6 JJt;
                JJt.noalias() = J * J.transpose();
                JJt.diagonal().array() += damp;
                v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
                q = pinocchio::integrate(model, q, v * DT);
                if (!(i % 10))
                    std::cout << i << ": error = " << err.transpose() << std::endl;
            }

            if (success) {
                std::cout << "Convergence achieved!" << std::endl;
            } else {
                std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
            }

            std::cout << "\nresult: " << q.transpose() << std::endl;
            std::cout << "\nfinal error: " << err.transpose() << std::endl;
            return 0;
        }
        */
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