#include "../include/controller.h"

Controller::Controller(ros::NodeHandle *n) : n_(*n) {
    param_sub = n_.subscribe("/joint_pd", 1, &Controller::paramCallback, this);
}

void Controller::paramCallback(const laikago_msgs::GainParam &msg) {
    for (int i = 0; i < 3; i++) {
        kp_[i] = msg.kp[i];
        kd_[i] = msg.kd[i];
    }
    for (int i = 0; i < 6; i++) {
        kt_[i] = msg.kt[i];
    }
}

void Controller::sendCommand() {
    kin_.update();
    est_.update();
    est_.publish();
    setMotorZero();

    // force from feet kinematics
    Eigen::Matrix<float, 12, 1> p_feet_desired;
    p_feet_desired.segment(0, 3) << +0.20, -0.16, -0.40;
    p_feet_desired.segment(3, 3) << +0.20, +0.16, -0.40;
    p_feet_desired.segment(6, 3) << -0.22, -0.16, -0.40;
    p_feet_desired.segment(9, 3) << -0.22, +0.16, -0.40;

    Eigen::Matrix<float, 12, 1> p_feet_error = Eigen::Matrix<float, 12, 1>::Zero();
    Eigen::Matrix<float, 12, 1> feet_force_kin = Eigen::Matrix<float, 12, 1>::Zero();

    p_feet_error = p_feet_desired - kin_.p_feet_;
    double kp_kin = 1500;
    feet_force_kin = fmin(time_ / 10.0, 1) * kp_kin * p_feet_error;

    // matrix of feet force to acceleration
    Eigen::Matrix<float, 3, 12> Mat_lin;
    Eigen::Matrix<float, 3, 12> Mat_rot;
    for (int i = 0; i < 4; i++) {
        Mat_lin.block(0, 3 * i, 3, 3) = Eigen::Matrix3f::Identity();
        Eigen::Vector3f foot_world = kin_.R_imu_ * kin_.p_feet_.segment(3 * i, 3);
        Mat_rot.block(0, 3 * i, 3, 3) = conjMatrix(foot_world);
    }

    Eigen::Matrix<float, 12, 12> Mat_force = Eigen::Matrix<float, 12, 12>::Identity();
    Eigen::DiagonalMatrix<float, 12> Mat_force_weight;
    Mat_force_weight.diagonal() = 1e-3 * Eigen::Matrix<float, 12, 1>::Ones();

    // reset in stance
    if (est_.isStance() and !is_stance_) {
        // todo: remove the for loop
        for (int i = 0; i < 100; i++) {
            est_.resetState();
        }
        yaw_offset_ = kin_.q_imu_[2];
        is_stance_ = true;
        std::cout << "reset and yaw offset: " << yaw_offset_ * 180 / M_PI << " degrees" << std::endl;
    } else if (!est_.isStance() and is_stance_) {
        is_stance_ = false;
        std::cout << "left in air" << std::endl;
    }

    // smooth control transition
    double switch_rate = 2e-3;
    if (is_stance_) {
        control_switch_weight_ = fmin(control_switch_weight_ + switch_rate, 1);
    } else {
        control_switch_weight_ = fmax(control_switch_weight_ - switch_rate, 0);
    }

    // acceleration
    Eigen::DiagonalMatrix<float, 6> kp_imu;
    kp_imu.diagonal() << 700 * 1e-1 * kt_[3], 700 * 1e-1 * kt_[3], 7000, 300, 300, 300;
    Eigen::DiagonalMatrix<float, 6> kd_imu;
    kd_imu.diagonal() << 0, 0, 0, 0, 0, 0;
    Eigen::Matrix<float, 6, 1> desired_pos_imu;
    desired_pos_imu << 0, 0, 0.4,
            0 + kp_[0] * M_PI / 180,
            0 + kp_[1] * M_PI / 180,
            0 + kp_[2] * M_PI / 180 + yaw_offset_;
    Eigen::Matrix<float, 6, 1> pos_imu;
    pos_imu << est_.worldState_.bodyPosition.x,
            est_.worldState_.bodyPosition.y,
            est_.worldState_.bodyPosition.z,
            kin_.q_imu_;
    Eigen::Matrix<float, 6, 1> vel_imu;
    vel_imu << est_.worldState_.bodySpeed.x,
            est_.worldState_.bodySpeed.y,
            est_.worldState_.bodySpeed.z,
            kin_.dq_imu_;
    Eigen::Matrix<float, 6, 1> acc_imu;
    acc_imu = fmin(time_ / 10.0, 1) * (kp_imu * (desired_pos_imu - pos_imu) - kd_imu * vel_imu);
    Eigen::Matrix<float, 12, 1> acc_force;
    acc_force = Eigen::Matrix<float, 12, 1>::Zero();
    Eigen::Matrix<float, 12, 1> p_feet_desired_g = p_feet_desired;
    for (int i = 0; i < 4; i++) {
//        acc_force.segment(3 * i, 3) << 0, 0, 0;
        p_feet_desired_g(3 * i + 2) = -0.35;
        acc_force.segment(3 * i, 3) = fmin(time_ / 10.0, 1) * -kp_kin * kin_.R_imu_ *
                                      (p_feet_desired_g.segment(3 * i, 3) - kin_.p_feet_.segment(3 * i, 3));
    }

    // linear optimization
    auto num_rows = Mat_lin.rows() + Mat_rot.rows() + Mat_force.rows();
    Eigen::MatrixXf opt_A(num_rows, 12);
    Eigen::MatrixXf opt_b(num_rows, 1);
    std::vector<Eigen::MatrixXf> vec_mat;

    vec_mat = swapLegs(0, Mat_lin, Mat_rot, Mat_force, Mat_force_weight);
    opt_A << vec_mat[0], vec_mat[1], vec_mat[3] * vec_mat[2];
    opt_b << acc_imu, vec_mat[3] * acc_force;
    Eigen::Matrix<float, 12, 1> grf_qp_1 = qp_solver_.solve(opt_A, opt_b);
//    grf_qp_1.segment(0, 3) = acc_force.segment(0, 3);
    grf_qp_1.segment(9, 3) = acc_force.segment(9, 3);

    vec_mat = swapLegs(1, Mat_lin, Mat_rot, Mat_force, Mat_force_weight);
    opt_A << vec_mat[0], vec_mat[1], vec_mat[3] * vec_mat[2];
    opt_b << acc_imu, vec_mat[3] * acc_force;
    Eigen::Matrix<float, 12, 1> grf_qp_2 = qp_solver_.solve(opt_A, opt_b);

    // swap legs
    double delta_transition = 0.1;
    double alpha_transition = tanh(sin(2 * M_PI * kt_[4] * time_) / delta_transition) / tanh(1 / delta_transition);
    alpha_transition = (alpha_transition + 1) / 2.0;
    Eigen::Matrix<float, 12, 1> grf_qp;
    grf_qp = alpha_transition * grf_qp_1 + (1 - alpha_transition) * grf_qp_2;
//    grf_qp = grf_qp_1;

    Eigen::Matrix<float, 12, 1> feet_force_grf = Eigen::Matrix<float, 12, 1>::Zero();
    for (int i = 0; i < 4; i++) {
        feet_force_grf.segment(3 * i, 3) = kin_.R_imu_.transpose() * -grf_qp.segment(3 * i, 3);
    }

    // merge kinematics and grf control
    Eigen::Matrix<float, 12, 1> feet_force = Eigen::Matrix<float, 12, 1>::Zero();
//    feet_force = feet_force_kin;
//    feet_force = (1 - control_switch_weight_) * feet_force_kin + control_switch_weight_ * feet_force_grf;
    feet_force = (1 - kt_[5]) * feet_force_kin + kt_[5] * feet_force_grf;

    // convert to torque
    Eigen::Matrix<float, 12, 1> motor_torque = Eigen::Matrix<float, 12, 1>::Zero();
    motor_torque = kin_.J_feet_.transpose() * feet_force;
    Eigen::Vector4f torque_hip_gravity;
    torque_hip_gravity << -0.86, +0.86, -0.86, +0.86;
    for (int i = 0; i < 4; i++) {
        motor_torque[i * 3 + 0] += torque_hip_gravity[i];
    }
    setTorque(motor_torque);
}

void Controller::sendCommandPD() {
    kin_.update();
    est_.update();
    est_.publish();
    setMotorZero();

    Eigen::Vector4f torque_hip_gravity;
    torque_hip_gravity << -0.86, +0.86, -0.86, +0.86;
    Eigen::Vector4f pos_hip;
    pos_hip << 0.0, 0.0, 0.0, 0.0;
    Eigen::Vector4f pos_thigh;
    pos_thigh << 0.67, 0.67, 0.67, 0.67;
    Eigen::Vector4f pos_calf;
    pos_calf << -1.3, -1.3, -1.3, -1.3;
    Eigen::Matrix<float, 12, 1> motor_torque = Eigen::Matrix<float, 12, 1>::Zero();
    for (int i = 0; i < 4; i++) {
        motor_torque[i * 3 + 0] = torque_hip_gravity[i];
        motor_torque[i * 3 + 0] += fmin(time_ / 10.0, 1) * 100 * (pos_hip[i] - kin_.q_motor_[i * 3 + 0]);
        motor_torque[i * 3 + 1] = fmin(time_ / 10.0, 1) * 120 * (pos_thigh[i] - kin_.q_motor_[i * 3 + 1]);
        motor_torque[i * 3 + 2] = fmin(time_ / 10.0, 1) * 60 * (pos_calf[i] - kin_.q_motor_[i * 3 + 2]);
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
        lowCmd.motorCmd[i * 3 + 0].positionStiffness = 0;
        lowCmd.motorCmd[i * 3 + 0].velocity = 0;
        lowCmd.motorCmd[i * 3 + 0].velocityStiffness = fmin(time_ / 5.0, 1) * (0.12 + kd_[0]);
        lowCmd.motorCmd[i * 3 + 1].position = PosStopF;
        lowCmd.motorCmd[i * 3 + 1].positionStiffness = 0;
        lowCmd.motorCmd[i * 3 + 1].velocity = 0;
        lowCmd.motorCmd[i * 3 + 1].velocityStiffness = fmin(time_ / 5.0, 1) * (0.04 + kd_[1]);
        lowCmd.motorCmd[i * 3 + 2].position = PosStopF;
        lowCmd.motorCmd[i * 3 + 2].positionStiffness = 0;
        lowCmd.motorCmd[i * 3 + 2].velocity = 0;
        lowCmd.motorCmd[i * 3 + 2].velocityStiffness = fmin(time_ / 5.0, 1) * (0.02 + kd_[2]);
    }
}

Eigen::Matrix3f Controller::conjMatrix(const Eigen::Vector3f &vec) {
    Eigen::Matrix3f mat;
    mat << 0, -vec[2], vec[1],
            vec[2], 0, -vec[0],
            -vec[1], vec[0], 0;
    return mat;
}

void Controller::setTrajectory(Eigen::Matrix<float, 12, 1> &p_feet_desired) {

}

std::vector<Eigen::MatrixXf>
Controller::swapLegs(int phase,
                     Eigen::Matrix<float, 3, 12> Mat_lin,
                     Eigen::Matrix<float, 3, 12> Mat_rot,
                     Eigen::Matrix<float, 12, 12> Mat_force,
                     Eigen::DiagonalMatrix<float, 12> Mat_force_weight) {
    Mat_lin.block(0, 0, 2, 12) *= 1e-1;

    if (phase == 0) {
//        Mat_lin.block(0, 0, 3, 3) *= 1e-3;
//        Mat_rot.block(0, 0, 3, 3) *= 1e-3;
//        Mat_force_weight.diagonal().segment(0, 3) *= 1e3;
        Mat_lin.block(0, 9, 3, 3) *= 1e-3;
        Mat_rot.block(0, 9, 3, 3) *= 1e-3;
        Mat_force_weight.diagonal().segment(9, 3) *= 1e3;
    } else if (phase == 1) {
//        Mat_lin.block(0, 3, 3, 3) *= 1e-3;
//        Mat_rot.block(0, 3, 3, 3) *= 1e-3;
//        Mat_force_weight.diagonal().segment(3, 3) *= 1e3;
//        Mat_lin.block(0, 6, 3, 3) *= 1e-3;
//        Mat_rot.block(0, 6, 3, 3) *= 1e-3;
//        Mat_force_weight.diagonal().segment(6, 3) *= 1e3;
    } else {
        std::cerr << "wrong phase: " << phase << std::endl;
    }
    return {Mat_lin, Mat_rot, Mat_force, Mat_force_weight};
}




