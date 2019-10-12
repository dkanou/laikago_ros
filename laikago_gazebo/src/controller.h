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
    void sendCommand() {
        kin_.update();
        bodyPoseEstimator_.update();
        setMotorZero();
        Eigen::Matrix<float, 12, 1> p_feet_desired;

        p_feet_desired.segment(0, 3) << 0.21, -0.14, -0.4;
        p_feet_desired.segment(3, 3) << 0.21, 0.14, -0.4;
        p_feet_desired.segment(6, 3) << -0.22, -0.14, -0.34;
        p_feet_desired.segment(9, 3) << -0.22, 0.14, -0.34;

        Eigen::Matrix<float, 12, 1> p_feet_error = p_feet_desired - kin_.p_feet_;
        Eigen::Matrix<float, 12, 1> feet_force = Eigen::Matrix<float, 12, 1>::Zero();
        feet_force = 2000.0 * p_feet_error;

        Eigen::Matrix<float, 12, 1> motor_torque = kin_.J_feet_.transpose() * feet_force;
        Eigen::Vector4f torque_hip_gravity;
        torque_hip_gravity << -0.86, 0.86, -0.86, 0.86;
        for (int i = 0; i < 4; i++) {
            motor_torque[i * 3 + 0] += torque_hip_gravity[i];
        }
        setTorque(motor_torque);

    }

    void sendCommandPD() {
        kin_.update();
        setMotorZero();
        Eigen::Vector4f torque_hip_gravity;
        torque_hip_gravity << -0.86, 0.86, -0.86, 0.86;
        Eigen::Vector4f pos_hip;
        pos_hip << 0.0, 0.0, 0.0, 0.0;
        Eigen::Vector4f pos_thigh;
        pos_thigh << 0.67, 0.67, 0.67, 0.67;
        Eigen::Vector4f pos_calf;
        pos_calf << -1.3, -1.3, -1.3, -1.3;
        Eigen::Matrix<float, 12, 1> motor_torque = Eigen::Matrix<float, 12, 1>::Zero();
        for (int i = 0; i < 4; i++) {
            motor_torque[i * 3 + 0] = torque_hip_gravity[i] + 70 * (pos_hip[i] - kin_.q_motor_[i * 3 + 0]);
            motor_torque[i * 3 + 1] = 180 * (pos_thigh[i] - kin_.q_motor_[i * 3 + 1]);
            motor_torque[i * 3 + 2] = 300 * (pos_calf[i] - kin_.q_motor_[i * 3 + 2]);
        }
        setTorque(motor_torque);
    }

    void setTime(const double &time) {
        time_ = time;
    }

    static void setMotorZero() {
        for (int i = 0; i < 4; i++) {
            lowCmd.motorCmd[i * 3 + 0].mode = 0x0A;
            lowCmd.motorCmd[i * 3 + 0].position = PosStopF;
            lowCmd.motorCmd[i * 3 + 0].positionStiffness = 0;
            lowCmd.motorCmd[i * 3 + 0].velocity = VelStopF;
            lowCmd.motorCmd[i * 3 + 0].velocityStiffness = 0;
            lowCmd.motorCmd[i * 3 + 0].torque = 0;
            lowCmd.motorCmd[i * 3 + 1].mode = 0x0A;
            lowCmd.motorCmd[i * 3 + 1].position = PosStopF;
            lowCmd.motorCmd[i * 3 + 1].positionStiffness = 0;
            lowCmd.motorCmd[i * 3 + 1].velocity = VelStopF;
            lowCmd.motorCmd[i * 3 + 1].velocityStiffness = 0;
            lowCmd.motorCmd[i * 3 + 1].torque = 0;
            lowCmd.motorCmd[i * 3 + 2].mode = 0x0A;
            lowCmd.motorCmd[i * 3 + 2].position = PosStopF;
            lowCmd.motorCmd[i * 3 + 2].positionStiffness = 0;
            lowCmd.motorCmd[i * 3 + 2].velocity = VelStopF;
            lowCmd.motorCmd[i * 3 + 2].velocityStiffness = 0;
            lowCmd.motorCmd[i * 3 + 2].torque = 0;
        }
    }

private:

    static void setTorque(const Eigen::Matrix<float, 12, 1> &motor_torque) {
        for (int i = 0; i < 12; i++) {
            lowCmd.motorCmd[i].torque = motor_torque[i];
        }
        for (int i = 0; i < 4; i++) {
            lowCmd.motorCmd[i * 3 + 0].position = PosStopF;
            lowCmd.motorCmd[i * 3 + 0].positionStiffness = 0; // 70
            lowCmd.motorCmd[i * 3 + 0].velocity = 0;
            lowCmd.motorCmd[i * 3 + 0].velocityStiffness = 3; // 3
            lowCmd.motorCmd[i * 3 + 1].position = PosStopF;
            lowCmd.motorCmd[i * 3 + 1].positionStiffness = 0; // 180
            lowCmd.motorCmd[i * 3 + 1].velocity = 0;
            lowCmd.motorCmd[i * 3 + 1].velocityStiffness = 4; // 8
            lowCmd.motorCmd[i * 3 + 2].position = PosStopF;
            lowCmd.motorCmd[i * 3 + 2].positionStiffness = 0; // 300
            lowCmd.motorCmd[i * 3 + 2].velocity = 0;
            lowCmd.motorCmd[i * 3 + 2].velocityStiffness = 2; // 15
        }
    }

    float time_{0};
    Kinematics kin_;
    BodyPoseEstimator bodyPoseEstimator_;
};


#endif //CATKIN_WS_CONTROLLER_H
