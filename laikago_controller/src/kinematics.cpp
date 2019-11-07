#include "../include/kinematics.h"

void Kinematics::update() {
    updateEigenState();
    updateKinematics();
    if (sim) {
        updateFootForce();
    }
}

void Kinematics::updateEigenState() {
    Eigen::Quaternionf quaternion(lowState.imu.quaternion[0],
                                  lowState.imu.quaternion[1],
                                  lowState.imu.quaternion[2],
                                  lowState.imu.quaternion[3]);
    R_imu_ = quaternion.toRotationMatrix();
    for (int i = 0; i < 3; i++) {
        q_imu_[i] = lowState.imu.rpy[i] * M_PI / 180.0f;
        dq_imu_[i] = lowState.imu.gyroscope[i];
    }
    for (int i = 0; i < 12; i++) {
        q_motor_[i] = lowState.motorState[i].position;
        dq_motor_[i] = lowState.motorState[i].velocity;
    }
}

void Kinematics::updateKinematics() {
    DM dm_q_motor{std::vector<double>(q_motor_.data(), q_motor_.size() + q_motor_.data())};
    DMDict feet_dict = f_kinBodyFeet_(DMDict{{"q", dm_q_motor}});
    Eigen::Map<Eigen::Matrix<double, 12, 1>> temp_p_feet(feet_dict["p_feet"].ptr(), 12);
    Eigen::Map<Eigen::Matrix<double, 12, 12>> temp_J_feet(feet_dict["J_feet"].ptr(), 12, 12);
    Eigen::Map<Eigen::Matrix<double, 12, 3>> temp_R_feet(feet_dict["R_feet"].ptr(), 12, 3);

    p_feet_ = temp_p_feet.cast<float>();
    J_feet_ = temp_J_feet.cast<float>();
    R_feet_ = temp_R_feet.cast<float>();
    dp_feet_ = J_feet_ * dq_motor_;
    for (int i = 0; i < 4; i++) {
        highState.footPosition2Body[i].x = p_feet_[i * 3 + 0];
        highState.footPosition2Body[i].y = p_feet_[i * 3 + 1];
        highState.footPosition2Body[i].z = p_feet_[i * 3 + 2];
        highState.footSpeed2Body[i].x = dp_feet_[i * 3 + 0];
        highState.footSpeed2Body[i].y = dp_feet_[i * 3 + 1];
        highState.footSpeed2Body[i].z = dp_feet_[i * 3 + 2];
    }
}

void Kinematics::updateFootForce() {
    for (int i = 0; i < 4; i++) {
        Eigen::Matrix<float, 3, 3> R_foot_world;
        R_foot_world = R_imu_ * R_feet_.block(i * 3, 0, 3, 3);
        Eigen::Matrix<float, 3, 1> eeForce_vec;
        eeForce_vec << lowState.eeForce[i].x, lowState.eeForce[i].y, lowState.eeForce[i].z;
        Eigen::Matrix<float, 3, 1> eeForce_vec_world;
        eeForce_vec_world = R_foot_world * eeForce_vec;
        lowState.footForce[i] = eeForce_vec_world[2];
    }
}

void Kinematics::setLowState(const laikago_msgs::LowState &RecvLowROS) {
    lowState.imu = RecvLowROS.imu;
    for (int i = 0; i < 12; i++) {
        lowState.motorState[i] = RecvLowROS.motorState[i+1];
    }
    lowState.footForce = RecvLowROS.footForce;
    lowState.tick = RecvLowROS.tick;
    lowState.wirelessRemote = RecvLowROS.wirelessRemote;
    lowState.crc = RecvLowROS.crc;
}

void Kinematics::setLowCmd(laikago_msgs::LowCmd &SendLowROS) {
    for (int i = 0; i < 12; i++) {
        SendLowROS.motorCmd[i+1] = lowCmd.motorCmd[i];
    }
}
