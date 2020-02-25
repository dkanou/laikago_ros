#ifndef CATKIN_WS_CONTROLLER_H
#define CATKIN_WS_CONTROLLER_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include "body.h"
#include "body_estimation.h"
#include "kinematics.h"
#include "laikago_msgs/GainParam.h"
#include "qp_solver.h"

using namespace casadi;
using laikago_model::lowCmd;

class LowPassFilter {
public:
    float firstOrder(float x_in, float fil_const) {
        low_pass_x_ = fil_const * x_in + (1 - fil_const) * low_pass_x_;
        return low_pass_x_;
    }

private:
    float low_pass_x_{0};
};

class Controller {
public:
    Controller(ros::NodeHandle *n);

    void paramCallback(const laikago_msgs::GainParam &msg);
    void sendCommand();
    void sendCommandPD();
    void setTime(const double &time);
    static void setMotorZero();
    static const std::vector<Eigen::Matrix<int, 4, 1>>& getGaitSet();

    static void __attribute__ ((used)) printMat(const Eigen::Matrix<float, 3, 12> &mat) {
        std::cout << mat << std::endl;
    }
    static void __attribute__ ((used)) printMat(const Eigen::MatrixXf &mat) {
        std::cout << mat << std::endl;
    }

private:
    void setTorque(const Eigen::Matrix<float, 12, 1> &motor_torque);
    void updateStance();
    Eigen::Matrix<float, 12, 1> getKinForce(const Eigen::Matrix<float, 12, 1>& p_feet_desired);
    void getDynMat(Eigen::Matrix<float, 3, 12> &Mat_lin,
                   Eigen::Matrix<float, 3, 12> &Mat_rot,
                   Eigen::Matrix<float, 12, 12> &Mat_force,
                   Eigen::DiagonalMatrix<float, 12> &Mat_force_weight);
    float getGroundWeight();
    Eigen::Matrix<float, 12, 1> getFpTarget();
    void getAccState(Eigen::Matrix<float, 6, 1>& acc_body, Eigen::Matrix<float, 12, 1>& acc_feet);
    int getGaitIndex(const std::vector<Eigen::Matrix<int, 4, 1>>&D_vec, const std::vector<float> &cost_vec);
    static Eigen::Matrix3f conjMatrix(const Eigen::Vector3f &vec);

    float time_{0};
    Kinematics kin_;
    BodyPoseEstimator est_;
    Eigen::Matrix<float, 12, 1> p_feet_default_;
    MultiQp multiQp_;
    unsigned int grf_index_{0};
    ros::NodeHandle &n_;
    ros::Subscriber param_sub;
    const float kp_kin_{1500};
    float kp_[3]{0, 0, 0};
    float kd_[3]{0, 0, 0};
    float kt_[6]{};
    bool is_stance_{false};
    float yaw_offset_{0};
    float control_switch_weight_{0};
    LowPassFilter lowPassFilter_[12];
    LowPassFilter lowPassFilterVel_[2];
};


#endif //CATKIN_WS_CONTROLLER_H
