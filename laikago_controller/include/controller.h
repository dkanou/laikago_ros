#ifndef CATKIN_WS_CONTROLLER_H
#define CATKIN_WS_CONTROLLER_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include "laikago_msgs/GainParam.h"
#include "body.h"
#include "kinematics.h"
#include "body_estimation.h"

using namespace casadi;
using laikago_model::lowCmd;

class Controller {
public:
    Controller(ros::NodeHandle *n);

    void paramCallback(const laikago_msgs::GainParam &msg);

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

    void setTorque(const Eigen::Matrix<float, 12, 1> &motor_torque);

    static Eigen::Matrix3f conjMatrix(const Eigen::Vector3f &vec);

    void setTrajectory(Eigen::Matrix<float, 12, 1> &p_feet_desired);

    void swapLegs(Eigen::Matrix<float, 3, 12> &Mat_lin,
                  Eigen::Matrix<float, 3, 12> &Mat_rot,
                  Eigen::DiagonalMatrix<float, 12> &Mat_force_weight);

    float time_{0};
    Kinematics kin_;
    BodyPoseEstimator est_;
    ros::NodeHandle &n_;
    ros::Subscriber param_sub;
    float kp_[3]{0, 0, 0};
    float kd_[3]{0, 0, 0};
    float kt_[6]{};
    bool is_stance_{false};
    float yaw_offset_{0};
    float control_switch_weight_{0};
};


#endif //CATKIN_WS_CONTROLLER_H
