#include "controller.h"

void Controller::sendCommand() {
    kin_.update();
    est_.update();
    est_.publish();
    setMotorZero();
    Eigen::Matrix<float, 12, 1> p_feet_desired;

    p_feet_desired.segment(0, 3) << 0.21, -0.14, -0.4;
    p_feet_desired.segment(3, 3) << 0.21, 0.14, -0.4;
    p_feet_desired.segment(6, 3) << -0.22, -0.14, -0.4;
    p_feet_desired.segment(9, 3) << -0.22, 0.14, -0.4;

    Eigen::Matrix<float, 12, 1> p_feet_error = p_feet_desired - kin_.p_feet_;
    Eigen::Matrix<float, 12, 1> feet_force = Eigen::Matrix<float, 12, 1>::Zero();
    feet_force = 2000.0 * p_feet_error;

    Eigen::Matrix<float, 12, 1> motor_torque = kin_.J_feet_.transpose() * feet_force;
    Eigen::Vector4f torque_hip_gravity;
    torque_hip_gravity << -0.86, 0.86, -0.86, 0.86;
    for (int i = 0; i < 4; i++) {
        motor_torque[i * 3 + 0] += torque_hip_gravity[i];
    }

    // matrix of feet force to acceleration
    Eigen::Matrix<float, 3, 12> Mat_lin;
    Eigen::Matrix<float, 3, 12> Mat_rot;
    for (int i = 0; i < 4; i++) {
        Mat_lin.block(0, 3 * i, 3, 3) = Eigen::Matrix3f::Identity();
        Eigen::Vector3f foot_world = kin_.R_imu_ * kin_.p_feet_.segment(3 * i, 3);
        Mat_rot.block(0, 3 * i, 3, 3) = conjMatrix(foot_world);
    }
    Eigen::Matrix<float, 12, 12> Mat_I = Eigen::Matrix<float, 12, 12>::Identity();

    // acceleration
    float kp_imu = 10000;
    float kd_imu = 100;
    Eigen::Matrix<float, 6, 1> desired_imu;
    desired_imu << 0, 0, 0.4, 0, 0, 0;
    Eigen::Matrix<float, 6, 1> pos_imu;
    pos_imu << est_.worldState_.bodyPosition.x * 1,
            est_.worldState_.bodyPosition.y * 1,
            est_.worldState_.bodyPosition.z,
            kin_.q_imu_;
    Eigen::Matrix<float, 6, 1> vel_imu;
    vel_imu << est_.worldState_.bodySpeed.x,
            est_.worldState_.bodySpeed.y,
            est_.worldState_.bodySpeed.z,
            kin_.dq_imu_;
    Eigen::Matrix<float, 6, 1> acc_imu;
    acc_imu = kp_imu * (desired_imu - pos_imu) - kd_imu * vel_imu;
    Eigen::Matrix<float, 12, 1> acc_zero;
    acc_zero = Eigen::Matrix<float, 12, 1>::Zero();

    // linear optimization
    Eigen::Matrix<float, 18, 12> opt_A;
    opt_A << Mat_lin, Mat_rot, Mat_I;
    Eigen::Matrix<float, 18, 1> opt_b;
    opt_b << acc_imu, acc_zero;
    Eigen::Matrix<float, 12, 1> grf;
    grf = opt_A.colPivHouseholderQr().solve(opt_b);

    // convert to torque
    Eigen::Matrix<float, 12, 1> feet_force2 = Eigen::Matrix<float, 12, 1>::Zero();
    for (int i = 0; i < 4; i++) {
        feet_force2.segment(3 * i, 3) = kin_.R_imu_.transpose() * -grf.segment(3 * i, 3);
    }
    Eigen::Matrix<float, 12, 1> motor_torque2 = kin_.J_feet_.transpose() * feet_force2;
    for (int i = 0; i < 4; i++) {
        motor_torque2[i * 3 + 0] += torque_hip_gravity[i];
    }

    Eigen::MatrixXf test_mat(3, 3);
    test_mat.setZero();

    if (time_ < 5.f)
        setTorque(motor_torque);
    else
        setTorque(motor_torque2);
}

void Controller::sendCommandPD() {
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

void Controller::setTime(const double &time) {
    time_ = time;
}

void Controller::setMotorZero() {
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

void Controller::setTorque(const Eigen::Matrix<float, 12, 1> &motor_torque) {
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

Eigen::Matrix3f Controller::conjMatrix(const Eigen::Vector3f &vec) {
    Eigen::Matrix3f mat;
    mat << 0, -vec[2], vec[1],
            vec[2], 0, -vec[0],
            -vec[1], vec[0], 0;
    return mat;
}
