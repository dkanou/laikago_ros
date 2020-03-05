#ifndef CATKIN_WS_KINEMATICS_H
#define CATKIN_WS_KINEMATICS_H

#include <casadi/casadi.hpp>
#include <Eigen/Geometry>
#include "body.h"
#include "eigen_def.h"

using namespace casadi;
using laikago_model::lowCmd;
using laikago_model::lowState;
using laikago_model::highState;

class Kinematics {
public:
    Kinematics() {
        std::string gen_path{std::getenv("MATLAB_GEN")};
        f_kinBodyFeet_ = external("kinBodyFeet", gen_path.append("/kinBodyFeet.so"));
        update();
    }

    void update();
    void updateEigenState();
    void updateKinematics();
    void updateFootForce();

    static void setLowState(const laikago_msgs::LowState &RecvLowROS);

    static void setLowCmd(laikago_msgs::LowCmd &SendLowROS);

    void __attribute__ ((used)) printFeet() {
        std::cout << "p_feet:\n" << p_feet_.transpose() << std::endl;
        std::cout << "J_feet:\n" << J_feet_ << std::endl;
        std::cout << "R_feet:\n" << R_feet_ << std::endl;
    }

    static void __attribute__ ((used)) printState() {
        std::cout << lowState << std::endl;
    }

    static void __attribute__ ((used)) printCmd() {
        std::cout << lowCmd << std::endl;
    }

    Vector3f q_imu_;
    Vector3f dq_imu_;
    Matrix3f R_imu_;
    Matrix3f R_yaw_;
    Vector12f q_motor_;
    Vector12f dq_motor_;
    Vector12f p_feet_;
    Vector12f dp_feet_;
    Matrix12f J_feet_;
    Eigen::Matrix<float, 12, 3> R_feet_;
    static bool sim;
private:
    Function f_kinBodyFeet_;
};

#endif //CATKIN_WS_KINEMATICS_H
