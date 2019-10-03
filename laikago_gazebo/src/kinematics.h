#ifndef CATKIN_WS_KINEMATICS_H
#define CATKIN_WS_KINEMATICS_H

#include <casadi/casadi.hpp>
#include <Eigen/Geometry>
#include "body.h"
#include "ros/ros.h"

using namespace casadi;
using laikago_model::lowState;

class Kinematics {
public:
    Kinematics() {
        std::string gen_path{std::getenv("MATLAB_GEN")};
        f_kinBodyFeet_ = external("kinBodyFeet", gen_path.append("/kinBodyFeet.so"));
        updateEigenState();
    }

    void updateEigenState() {
        Eigen::Quaternionf quaternion(lowState.imu.quaternion[0],
                                      lowState.imu.quaternion[1],
                                      lowState.imu.quaternion[2],
                                      lowState.imu.quaternion[3]);
        R_imu_ = quaternion.toRotationMatrix();
        for (int i = 0; i < 3; i++) {
            q_imu_[i] = lowState.imu.rpy[i];
        }
        for (int i = 0; i < 12; i++) {
            q_motor_[i] = lowState.motorState[i].position;
            dq_motor_[i] = lowState.motorState[i].velocity;
        }
    }

    void updateKinematics() {
        DM dm_q_motor{std::vector<double>(q_motor_.data(), q_motor_.size() + q_motor_.data())};
        DMDict feet_dict = f_kinBodyFeet_(DMDict{{"q", dm_q_motor}});
        Eigen::Map<Eigen::Matrix<double, 12, 1>> temp_p_feet(feet_dict["p_feet"].ptr(), 12);
        Eigen::Map<Eigen::Matrix<double, 12, 12>> temp_J_feet(feet_dict["J_feet"].ptr(), 12, 12);

        p_feet_ = temp_p_feet.cast<float>();
        J_feet_ = temp_J_feet.cast<float>();

        std::cout << feet_dict["p_feet"] << std::endl;
        std::cout << p_feet_.transpose() << std::endl;
        std::cout << feet_dict["J_feet"] << std::endl;
        std::cout << J_feet_ << std::endl;
    }

    void readSensors() {
        updateEigenState();
        updateKinematics();
    }

    static void __attribute__ ((used)) printState() {
        std::cout << lowState << std::endl;
    }

private:
    Eigen::Matrix<float, 3, 1> q_imu_;
    Eigen::Matrix<float, 3, 3> R_imu_;
    Eigen::Matrix<float, 12, 1> q_motor_;
    Eigen::Matrix<float, 12, 1> dq_motor_;
    Eigen::Matrix<float, 12, 1> p_feet_;
    Eigen::Matrix<float, 12, 12> J_feet_;
    Function f_kinBodyFeet_;
};

#endif //CATKIN_WS_KINEMATICS_H
