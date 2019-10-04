#ifndef CATKIN_WS_CONTROLLER_H
#define CATKIN_WS_CONTROLLER_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include "body.h"
#include "kinematics.h"

using namespace casadi;
using laikago_model::lowCmd;

class Controller {
public:
    void sendCommand() {
        kin_.update();
        setMotorZero();
        Eigen::Vector4f hip_gravity;
        hip_gravity << -0.86, 0.86, -0.86, 0.86;
        Eigen::Vector4f pos_hip;
        pos_hip << 0.0, 0.0, 0.0, 0.0;
        Eigen::Vector4f pos_thigh;
        pos_thigh << 0.67, 0.67, 0.67, 0.67;
        Eigen::Vector4f pos_calf;
        pos_calf << -1.3, -1.3, -1.3, -1.3;

        for (int i = 0; i < 4; i++) {
            lowCmd.motorCmd[i * 3 + 0].torque = hip_gravity[i] + 70 * (pos_hip[i] - kin_.q_motor_[i * 3 + 0]);
            lowCmd.motorCmd[i * 3 + 1].torque = 180 * (pos_thigh[i] - kin_.q_motor_[i * 3 + 1]);
            lowCmd.motorCmd[i * 3 + 2].torque = 300 * (pos_calf[i] - kin_.q_motor_[i * 3 + 2]);
        }
        for (int i = 0; i < 4; i++) {
            lowCmd.motorCmd[i * 3 + 0].position = pos_hip[i];
            lowCmd.motorCmd[i * 3 + 0].positionStiffness = 0; // 70
            lowCmd.motorCmd[i * 3 + 0].velocity = 0;
            lowCmd.motorCmd[i * 3 + 0].velocityStiffness = 3; // 3
            lowCmd.motorCmd[i * 3 + 1].position = pos_thigh[i];
            lowCmd.motorCmd[i * 3 + 1].positionStiffness = 0; // 180
            lowCmd.motorCmd[i * 3 + 1].velocity = 0;
            lowCmd.motorCmd[i * 3 + 1].velocityStiffness = 4; // 8
            lowCmd.motorCmd[i * 3 + 2].position = pos_calf[i];
            lowCmd.motorCmd[i * 3 + 2].positionStiffness = 0; // 300
            lowCmd.motorCmd[i * 3 + 2].velocity = 0;
            lowCmd.motorCmd[i * 3 + 2].velocityStiffness = 2; // 15
        }
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
    double time_{0};
    Kinematics kin_;
};


#endif //CATKIN_WS_CONTROLLER_H
