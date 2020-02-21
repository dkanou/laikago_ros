#include "../include/controller.h"

Controller::Controller(ros::NodeHandle *n) : n_(*n) {
    param_sub = n_.subscribe("/joint_pd", 1, &Controller::paramCallback, this);
    p_feet_default_.segment(0, 3) << +0.20, -0.16, -0.40;
    p_feet_default_.segment(3, 3) << +0.20, +0.16, -0.40;
    p_feet_default_.segment(6, 3) << -0.22, -0.16, -0.40;
    p_feet_default_.segment(9, 3) << -0.22, +0.16, -0.40;
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
    est_.update(); // TODO: improve the state estimation speed
    est_.publish();
    updateStance();
    setMotorZero();

    // force from feet kinematics
    Eigen::Matrix<float, 12, 1> p_feet_desired(p_feet_default_);
    Eigen::Matrix<float, 12, 1> feet_force_kin = getKinForce(p_feet_desired);

    // matrix of feet force to acceleration
    Eigen::Matrix<float, 3, 12> Mat_lin;
    Eigen::Matrix<float, 3, 12> Mat_rot;
    Eigen::Matrix<float, 12, 12> Mat_force;
    Eigen::DiagonalMatrix<float, 12> Mat_force_weight;
    getDynMat(Mat_lin, Mat_rot, Mat_force, Mat_force_weight);

    // acceleration
    Eigen::Matrix<float, 6, 1> acc_body;
    Eigen::Matrix<float, 12, 1> acc_feet;
    getAccState(acc_body, acc_feet);

    // generate smooth square function for transition
    float smooth_rate = 0.2;
    float gait_transition = tanhf(sinf(2 * M_PI * kt_[4] * time_) / smooth_rate) / tanhf(1 / smooth_rate);
    gait_transition = (gait_transition + 1) / 2.0f;
    float swing_transition = sinf(2 * M_PI * (kt_[4] * time_ - 0.1));
    swing_transition = (swing_transition + 1) / 2.0f;

    // linear optimization
    auto num_rows = Mat_lin.rows() + Mat_rot.rows() + Mat_force.rows();
    Eigen::MatrixXf opt_A(num_rows, 12);
    Eigen::MatrixXf opt_b(num_rows, 1);
    std::vector<Eigen::MatrixXf> vec_mat;

    vec_mat = swapLegs(0, Mat_lin, Mat_rot, Mat_force, Mat_force_weight);
    opt_A << vec_mat[0], vec_mat[1], vec_mat[3] * vec_mat[2];
    opt_b << acc_body, vec_mat[3] * acc_feet;
    Eigen::Matrix<float, 12, 1> grf_qp_0 = qp_solver0_.solve(opt_A, opt_b);
    grf_qp_0.segment(0, 3) = acc_feet.segment(0, 3) * (1 - swing_transition);
    grf_qp_0.segment(9, 3) = acc_feet.segment(9, 3) * (1 - swing_transition);

    vec_mat = swapLegs(1, Mat_lin, Mat_rot, Mat_force, Mat_force_weight);
    opt_A << vec_mat[0], vec_mat[1], vec_mat[3] * vec_mat[2];
    opt_b << acc_body, vec_mat[3] * acc_feet;
    Eigen::Matrix<float, 12, 1> grf_qp_1 = qp_solver1_.solve(opt_A, opt_b);
    grf_qp_1.segment(3, 3) = acc_feet.segment(3, 3) * swing_transition;
    grf_qp_1.segment(6, 3) = acc_feet.segment(6, 3) * swing_transition;

    // swap legs
    Eigen::Matrix<float, 12, 1> grf_qp;
    grf_qp = (1 - gait_transition) * grf_qp_0 + gait_transition * grf_qp_1;
//    grf_qp = grf_qp_1;

    Eigen::Matrix<float, 12, 1> feet_force_grf;
    for (int i = 0; i < 4; i++) {
        feet_force_grf.segment(3 * i, 3) = kin_.R_imu_.transpose() * -grf_qp.segment(3 * i, 3);
    }

    // merge kinematics and grf control
    Eigen::Matrix<float, 12, 1> feet_force;
//    feet_force = feet_force_kin;
    feet_force = (1 - kt_[5]) * feet_force_kin + kt_[5] * feet_force_grf;
//    double ground_weight = getGroundWeight();
//    feet_force = (1 - ground_weight) * feet_force_kin + ground_weight * feet_force_grf;

    // convert to torque
    Eigen::Matrix<float, 12, 1> motor_torque;
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
    time_ = float(time);
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
        lowCmd.motorCmd[i * 3 + 0].velocityStiffness = fmin(time_ / 5.0, 1) * (0.08);
        lowCmd.motorCmd[i * 3 + 1].position = PosStopF;
        lowCmd.motorCmd[i * 3 + 1].positionStiffness = 0;
        lowCmd.motorCmd[i * 3 + 1].velocity = 0;
        lowCmd.motorCmd[i * 3 + 1].velocityStiffness = fmin(time_ / 5.0, 1) * (0.04);
        lowCmd.motorCmd[i * 3 + 2].position = PosStopF;
        lowCmd.motorCmd[i * 3 + 2].positionStiffness = 0;
        lowCmd.motorCmd[i * 3 + 2].velocity = 0;
        lowCmd.motorCmd[i * 3 + 2].velocityStiffness = fmin(time_ / 5.0, 1) * (0.02);
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
        Mat_lin.block(0, 0, 3, 3) *= 1e-3;
        Mat_rot.block(0, 0, 3, 3) *= 1e-3;
        Mat_force_weight.diagonal().segment(0, 3) *= 1e3;
        Mat_lin.block(0, 9, 3, 3) *= 1e-3;
        Mat_rot.block(0, 9, 3, 3) *= 1e-3;
        Mat_force_weight.diagonal().segment(9, 3) *= 1e3;
    } else if (phase == 1) {
        Mat_lin.block(0, 3, 3, 3) *= 1e-3;
        Mat_rot.block(0, 3, 3, 3) *= 1e-3;
        Mat_force_weight.diagonal().segment(3, 3) *= 1e3;
        Mat_lin.block(0, 6, 3, 3) *= 1e-3;
        Mat_rot.block(0, 6, 3, 3) *= 1e-3;
        Mat_force_weight.diagonal().segment(6, 3) *= 1e3;
    } else {
        std::cerr << "wrong phase: " << phase << std::endl;
    }
    return {Mat_lin, Mat_rot, Mat_force, Mat_force_weight};
}

void Controller::updateStance() {
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
}

Eigen::Matrix<float, 12, 1> Controller::getKinForce(const Eigen::Matrix<float, 12, 1> &p_feet_desired) {
    Eigen::Matrix<float, 12, 1> p_feet_error = p_feet_desired - kin_.p_feet_;
    Eigen::Matrix<float, 12, 1> feet_force_kin = fmin(time_ / 10.0, 1) * kp_kin_ * p_feet_error;
    return feet_force_kin;
}

void Controller::getDynMat(Eigen::Matrix<float, 3, 12> &Mat_lin,
                           Eigen::Matrix<float, 3, 12> &Mat_rot,
                           Eigen::Matrix<float, 12, 12> &Mat_force,
                           Eigen::DiagonalMatrix<float, 12> &Mat_force_weight) {
    for (int i = 0; i < 4; i++) {
        Mat_lin.block(0, 3 * i, 3, 3) = Eigen::Matrix3f::Identity();
        Eigen::Vector3f foot_world = kin_.R_imu_ * kin_.p_feet_.segment(3 * i, 3);
        Mat_rot.block(0, 3 * i, 3, 3) = conjMatrix(foot_world);
    }
    Mat_force = Eigen::Matrix<float, 12, 12>::Identity();
    Mat_force_weight.diagonal() = 1e-3 * Eigen::Matrix<float, 12, 1>::Ones();

}

float Controller::getGroundWeight() {
    static float ground_weight{0};
    float switch_rate = 2e-3;
    if (is_stance_) {
        ground_weight = fmin(ground_weight + switch_rate, 1);
    } else {
        ground_weight = fmax(ground_weight - switch_rate, 0);
    }
    return ground_weight;
}

void Controller::getAccState(Eigen::Matrix<float, 6, 1> &acc_body, Eigen::Matrix<float, 12, 1> &acc_feet) {
    Eigen::DiagonalMatrix<float, 6> kp_body, kd_body;
    kp_body.diagonal() << 700 * 0, 700 * 0, 7000, 300, 300, 300;
    kd_body.diagonal() << 30 + kt_[2], 30 + kt_[2], 0, 0, 0, 0.5 + kt_[3];
    Eigen::Matrix<float, 6, 1> desired_pos_body, pos_body, vel_body;
    Eigen::Matrix<float, 3, 1> bodySpeed_offset;
    desired_pos_body << 0, 0, 0.4,
            0 + kp_[0] * M_PI / 180,
            0 + kp_[1] * M_PI / 180,
            0 + kp_[2] * M_PI / 180 + yaw_offset_;
    pos_body << est_.worldState_.bodyPosition.x,
            est_.worldState_.bodyPosition.y,
            est_.worldState_.bodyPosition.z,
            kin_.q_imu_;
    vel_body << est_.worldState_.bodySpeed.x,
            est_.worldState_.bodySpeed.y,
            est_.worldState_.bodySpeed.z,
            kin_.dq_imu_;
    bodySpeed_offset << 0.05 + kt_[0], 0.06 + kt_[1], 0;
    vel_body.segment(0, 3) += kin_.R_yaw_ * bodySpeed_offset;
    acc_body = fmin(time_ / 10.0, 1) * (kp_body * (desired_pos_body - pos_body) - kd_body * vel_body);

    // foot placement
    Eigen::Matrix<float, 12, 1> p_feet_desired_fp(p_feet_default_);
    for (int i = 0; i < 4; i++) {
        Eigen::Matrix<float, 3, 1> p_feet_delta;
        p_feet_delta << est_.worldState_.bodySpeed.x * (0.1 + kd_[0]),
                est_.worldState_.bodySpeed.y * (0.2 + kd_[1]),
                0.07;
        p_feet_desired_fp.segment(3 * i, 3) += kin_.R_yaw_.transpose() * p_feet_delta;
    }
    auto feet_force_fp = getKinForce(p_feet_desired_fp);
    for (int i = 0; i < 4; i++) {
        // the grf is opposite of the motor force
        acc_feet.segment(3 * i, 3) = kin_.R_imu_ * -feet_force_fp.segment(3 * i, 3);
    }
}




