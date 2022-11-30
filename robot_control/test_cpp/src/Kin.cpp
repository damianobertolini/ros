#include <Eigen/Dense>


class Kin{
    Eigen::Vector<double, 7> arm_l; //lunghezza bracci
    //Eigen::Vector<double, 6> joint_q; //rotazione joints
    
    public:
        Kin(){
            arm_l << 0.089159, 0.13585, 0.425, 0.1197, 0.39225, 0.094, 0.068;
            Eigen::Vector<double, 6> Th(0.089159, 0, 0, 0.10915, 0.09465, 0.0823);
        }
        Eigen::Matrix4d get_T0e(Eigen::Vector<double, 6> Th) {
            Eigen::Matrix4d T01 = T01f(Th(0));
            Eigen::Matrix4d T12 = T12f(Th(1));
            Eigen::Matrix4d T23 = T23f(Th(2));
            Eigen::Matrix4d T34 = T34f(Th(3));
            Eigen::Matrix4d T4e = T4ef(Th(4));
            //Eigen::Matrix4d T65m = T65f(Th(5));

            Eigen::Matrix4d T0e = T01*T12*T23*T34*T4e;
            return T0e;
        }
        private:
        Eigen::Matrix4d T01f(double th1){
            Eigen::Matrix4d tmp;
            Eigen::Matrix4d tmp2;
            tmp << 1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, arm_l[0],
                        0, 0, 0, 1;
            tmp2 << cos(th1), -sin(th1), 0, 0,
                    sin(th1), cos(th1),  0, 0,
                    0,               0,              1, 0,
                    0,               0,              0, 1;
            return tmp*tmp2;
            //rotazione e 
        }
        Eigen::Matrix4d T12f(double th2){
            Eigen::Matrix4d tmp;
            Eigen::Matrix4d tmp2;
            tmp <<  0, 0, 1,  0,
                    0, 1, 0, arm_l[1],
                    -1, 0, 0,  0,
                    0, 0, 0,  1;
            tmp2 << cos(th2), 0, sin(th2),        0,
                    0, 1,            0,        0,
                    sin(th2), 0, cos(th2), 0,
                    0, 0,              0, 1;
            return tmp*tmp2;
        }
        Eigen::Matrix4d T23f(double th3){
            
            Eigen::Matrix4d tmp;
            Eigen::Matrix4d tmp2;
            tmp <<  1,0,0,0,
                    0, 1, 0, (0-arm_l[3]),//negativo
                    0, 0, 1,  arm_l[2],
                    0, 0, 0,  1;
            tmp2 << cos(th3), 0, sin(th3),        0,
                    0, 1,            0,        0,
                    -sin(th3), 0, cos(th3), 0,
                    0, 0,              0, 1;

            return tmp*tmp2;
        }
        Eigen::Matrix4d T34f(double th4){
            Eigen::Matrix4d tmp;
            Eigen::Matrix4d tmp2;
            tmp <<  0,0,1,0,
                    0, 1, 0, 0,
                    -1, 0, 0,  arm_l[4],
                    0, 0, 0,  1;
            tmp2 << cos(th4), 0, sin(th4),        0,
                    0, 1,            0,        0,
                    -sin(th4), 0, cos(th4), 0,
                    0, 0,              0, 1;

            return tmp*tmp2;
        }
        Eigen::Matrix4d T4ef(double th5){
            Eigen::Matrix4d tmp;
            Eigen::Matrix4d tmp2;
            tmp <<  0,-1,0,0,
                    1, 0, 0, arm_l[5],
                    0, 0, 1,  arm_l[6],
                    0, 0, 0,  1;

            return tmp;//solo traslazione end effector
        }
        


};