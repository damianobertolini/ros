#include <pinocchio/fwd.hpp>
#include <Eigen/Dense>
#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/parsers/sample-models.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <iostream>
#include <vector>

using namespace pinocchio;
using namespace std;

class Kin {
    Eigen::Vector < double, 7 > arm_l; //lunghezza bracci
    //Eigen::Vector<double, 6> joint_q; //rotazione joints

    public:
        Kin() {
            arm_l << 0.089159, 0.13585, 0.425, 0.1197, 0.39225, 0.094, 0.068;
            Eigen::Vector < double, 6 > Th(0.089159, 0, 0, 0.10915, 0.09465, 0.0823);
        }
    Eigen::Matrix4d get_T0e(Eigen::Vector < double, 6 > Th) {
        Eigen::Matrix4d T01 = T01f(Th(0));
        Eigen::Matrix4d T12 = T12f(Th(1));
        Eigen::Matrix4d T23 = T23f(Th(2));
        Eigen::Matrix4d T34 = T34f(Th(3));
        Eigen::Matrix4d T4e = T4ef(Th(4));
        //Eigen::Matrix4d T65m = T65f(Th(5));

        Eigen::Matrix4d T0e = T01 * T12 * T23 * T34 * T4e;
        return T0e;
    }
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
        const std::string urdf_filename = std::string("$HOME/ros_ws/src/robo/ros/robot_urdf/generated_urdf/ur5.urdf");

        // Load the urdf model
        Model model;
        pinocchio::urdf::buildModel(urdf_filename, model);
        std::cout << "model name: " << model.name << std::endl;

        // Create data required by the algorithms
        Data data(model);

        // Sample a random configuration
        Eigen::VectorXd q = randomConfiguration(model);
        //-----------

        std::vector<double> a= { 0.02699, -0.66813, -0.32761, -0.0235,   0.02016,  0.};
        q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(a.data(), a.size());        //--------
        std::cout << "configurazione random: " << q << endl;

        std::cout << "q: " << q.transpose() << std::endl;

        // Perform the forward kinematics over the kinematic tree
        forwardKinematics(model, data, q);

        // Print out the placement of each joint of the kinematic tree
        for (JointIndex joint_id = 0; joint_id < (JointIndex) model.njoints; ++joint_id)
            std::cout << std::setw(24) << std::left <<
            model.names[joint_id] << ": " <<
            std::fixed << std::setprecision(2) <<
            data.oMi[joint_id].translation().transpose() <<
            std::endl;

        return 0;

    }
    int test_pinocchio3() {
        
            pinocchio::Model model;
            pinocchio::buildModels::manipulator(model);
            pinocchio::Data data(model);

            const int JOINT_ID = 6;
            const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1., 0., 1.));

            Eigen::VectorXd q = pinocchio::neutral(model);
            std::vector<double> a= { 0.02699, -0.66813, -0.32761, -0.0235,   0.02016,  0.};
            q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(a.data(), a.size());


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

        private:
            Eigen::Matrix4d T01f(double th1) {
                Eigen::Matrix4d tmp;
                Eigen::Matrix4d tmp2;
                tmp << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, arm_l[0],
                    0, 0, 0, 1;
                tmp2 << cos(th1), -sin(th1), 0, 0,
                    sin(th1), cos(th1), 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;
                return tmp * tmp2;
                //rotazione e 
            }
        Eigen::Matrix4d T12f(double th2) {
            Eigen::Matrix4d tmp;
            Eigen::Matrix4d tmp2;
            tmp << 0, 0, 1, 0,
                0, 1, 0, arm_l[1],
                -1, 0, 0, 0,
                0, 0, 0, 1;
            tmp2 << cos(th2), 0, sin(th2), 0,
                0, 1, 0, 0,
                sin(th2), 0, cos(th2), 0,
                0, 0, 0, 1;
            return tmp * tmp2;
        }
        Eigen::Matrix4d T23f(double th3) {

            Eigen::Matrix4d tmp;
            Eigen::Matrix4d tmp2;
            tmp << 1, 0, 0, 0,
                0, 1, 0, (0 - arm_l[3]), //negativo
                0, 0, 1, arm_l[2],
                0, 0, 0, 1;
            tmp2 << cos(th3), 0, sin(th3), 0,
                0, 1, 0, 0,
                -sin(th3), 0, cos(th3), 0,
                0, 0, 0, 1;

            return tmp * tmp2;
        }
        Eigen::Matrix4d T34f(double th4) {
            Eigen::Matrix4d tmp;
            Eigen::Matrix4d tmp2;
            tmp << 0, 0, 1, 0,
                0, 1, 0, 0,
                -1, 0, 0, arm_l[4],
                0, 0, 0, 1;
            tmp2 << cos(th4), 0, sin(th4), 0,
                0, 1, 0, 0,
                -sin(th4), 0, cos(th4), 0,
                0, 0, 0, 1;

            return tmp * tmp2;
        }
        Eigen::Matrix4d T4ef(double th5) {
            Eigen::Matrix4d tmp;
            Eigen::Matrix4d tmp2;
            tmp << 0, -1, 0, 0,
                1, 0, 0, arm_l[5],
                0, 0, 1, arm_l[6],
                0, 0, 0, 1;

            return tmp; //solo traslazione end effector
        }

    };