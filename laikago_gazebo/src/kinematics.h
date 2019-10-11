#ifndef CATKIN_WS_KINEMATICS_H
#define CATKIN_WS_KINEMATICS_H

#include <casadi/casadi.hpp>
#include <Eigen/Geometry>
#include "body.h"

using namespace casadi;
using laikago_model::lowState;
using laikago_model::highState;

class Kinematics {
public:
    Kinematics() {
        std::string gen_path{std::getenv("MATLAB_GEN")};
        f_kinBodyFeet_ = external("kinBodyFeet", gen_path.append("/kinBodyFeet.so"));
        update();
    }

    void update() {
        updateEigenState();
        updateKinematics();
        updateFootForce();
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
        Eigen::Map<Eigen::Matrix<double, 12, 3>> temp_R_feet(feet_dict["R_feet"].ptr(), 12, 3);

        p_feet_ = temp_p_feet.cast<float>();
        J_feet_ = temp_J_feet.cast<float>();
        R_feet_ = temp_R_feet.cast<float>();
        for (int i = 0; i < 4; i++) {
            highState.footPosition2Body[i].x = p_feet_[i * 3 + 0];
            highState.footPosition2Body[i].y = p_feet_[i * 3 + 1];
            highState.footPosition2Body[i].z = p_feet_[i * 3 + 2];
        }
    }

    void updateFootForce() {
        for (int i = 0; i < 4; i++) {
            Eigen::Matrix<float, 3, 3> R_foot_world;
            R_foot_world = R_imu_ * R_feet_.block(i*3, 0, 3, 3);
            Eigen::Matrix<float, 3, 1> eeForce_vec;
            eeForce_vec << lowState.eeForce[i].x, lowState.eeForce[i].y, lowState.eeForce[i].z;
            Eigen::Matrix<float, 3, 1> eeForce_vec_world;
            eeForce_vec_world = R_foot_world * eeForce_vec;
            lowState.footForce[i] = eeForce_vec_world[2];
        }
    }

    friend class Controller;

    void __attribute__ ((used)) printFeet() {
        std::cout << "p_feet:\n" << p_feet_.transpose() << std::endl;
        std::cout << "J_feet:\n" << J_feet_ << std::endl;
        std::cout << "R_feet:\n" << R_feet_ << std::endl;
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
    Eigen::Matrix<float, 12, 3> R_feet_;
    Function f_kinBodyFeet_;
};

#endif //CATKIN_WS_KINEMATICS_H
