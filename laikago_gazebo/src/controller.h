#ifndef CATKIN_WS_CONTROLLER_H
#define CATKIN_WS_CONTROLLER_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include "body.h"
#include "kinematics.h"
#include "body_estimation.h"

using namespace casadi;
using laikago_model::lowCmd;

class Controller {
public:
    void sendCommand();

    void sendCommandPD();

    void setTime(const double &time);

    static void setMotorZero();

    static void __attribute__ ((used)) printMat(const Eigen::Matrix<float, 3, 12> &mat) {
        std::cout << mat << std::endl;
    }

    static void __attribute__ ((used)) printMat(const Eigen::MatrixXf &mat) {
        std::cout << mat << std::endl;
    }

private:

    static void setTorque(const Eigen::Matrix<float, 12, 1> &motor_torque);

    static Eigen::Matrix3f conjMatrix(const Eigen::Vector3f &vec);

    float time_{0};
    Kinematics kin_;
    BodyPoseEstimator est_;
};


#endif //CATKIN_WS_CONTROLLER_H
