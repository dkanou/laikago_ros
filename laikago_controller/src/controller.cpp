#include "../include/controller.h"

Controller::Controller(ros::NodeHandle *n) : n_(*n), multiQp_(getGaitSet()) {
    param_sub_ = n_.subscribe("/joint_pd", 1, &Controller::paramCallback, this);
    controlState_pub_ = n_.advertise<laikago_msgs::ControlState>("/control_state", 1);
    p_feet_default_.segment(0, 3) << +0.20, -0.14, -0.40;
    p_feet_default_.segment(3, 3) << +0.20, +0.14, -0.40;
    p_feet_default_.segment(6, 3) << -0.22, -0.14, -0.40;
    p_feet_default_.segment(9, 3) << -0.22, +0.14, -0.40;
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
    vel_x_ = lpFilterVel_[0].firstOrder(est_.worldState_.bodySpeed.x, 0.02); // no filtering
    vel_y_ = lpFilterVel_[1].firstOrder(est_.worldState_.bodySpeed.y, 0.02);
    est_.publish();
    controlState_.bodySpeed.x = est_.worldState_.bodySpeed.x;
    controlState_.bodySpeed.y = est_.worldState_.bodySpeed.y;
    controlState_.bodySpeedFiltered.x = vel_x_;
    controlState_.bodySpeedFiltered.y = vel_y_;
    controlState_pub_.publish(controlState_);
    updateStance();
    setMotorZero();

    // force from feet kinematics
    Vector12f p_feet_desired(p_feet_default_);
    p_feet_desired(0 + 1) -= 0.03;
    p_feet_desired(3 + 1) += 0.03;
    p_feet_desired(6 + 1) -= 0.03;
    p_feet_desired(9 + 1) += 0.03;
    Vector12f feet_force_kin = getKinForce(p_feet_desired);

    // matrix of feet force to acceleration
    Eigen::Matrix<float, 3, 12> Mat_lin;
    Eigen::Matrix<float, 3, 12> Mat_rot;
    getDynMat(Mat_lin, Mat_rot);

    // acceleration
    Vector6f acc_body;
    getAccState(acc_body);

    // linear optimization
    Eigen::Matrix<float, 6, 12> opt_A;
    Eigen::Matrix<float, 6, 1> opt_b;
    opt_A << Mat_lin, Mat_rot;
    opt_A.block(0, 0, 2, 12) *= 1e-1;   //todo: find another way to adjust
    opt_b = acc_body;

    // feet pattern
    const auto D_vec = getGaitSet();
    multiQp_.multiSolve(opt_A, opt_b, grf_index_);
    const auto grf_qp_vec = multiQp_.getGrf();
    const auto cost_vec = multiQp_.getCost();

    static unsigned int s_ptime{0};
    s_ptime = (s_ptime+1)%sample_t_;
    if (s_ptime==0){
        grf_index_ = getGaitIndex(D_vec, cost_vec);
    }

//    static unsigned int s_ptime{0};
//    s_ptime = (s_ptime + 1) % (sample_t_ * 2);
//    if (s_ptime == 0)
//        grf_index_ = 1; // trot1
//    if (s_ptime == sample_t_)
//        grf_index_ = 2; // trot2

//    grf_index_ = 0;

    // apply index
    auto grf_qp = grf_qp_vec[grf_index_];
    auto D = D_vec[grf_index_];
    Vector12f p_feet_desired_fp = getFpTarget();
    Vector12f feet_force_fp = getFpForce(p_feet_desired_fp);

    // grf to feet force
    Vector12f feet_force_in = Vector12f::Zero();
    Vector12f feet_force_grf = Vector12f::Zero();
    for (int i = 0; i < 4; i++) {
        if (D[i] == 1)
            feet_force_in.segment(3 * i, 3) = feet_force_fp.segment(3 * i, 3);
        else
            feet_force_in.segment(3 * i, 3) = kin_.R_imu_.transpose() * -grf_qp.segment(3 * i, 3);
    }
    float fil_const{0.1};
    for (int i = 0; i < 12; i++) {
        // todo: remove the filter, find a better transition
        feet_force_grf(i) = lpFilterGrf_[i].firstOrder(feet_force_in(i), fil_const);
    }

    // merge kinematics and grf control
    Vector12f feet_force;
//    feet_force = feet_force_kin;
    feet_force = (1 - kt_[5]) * feet_force_kin + kt_[5] * feet_force_grf;
//    double ground_weight = getGroundWeight();
//    feet_force = (1 - ground_weight) * feet_force_kin + ground_weight * feet_force_grf;

    // convert to torque
    Vector12f motor_torque;
    motor_torque = kin_.J_feet_.transpose() * feet_force;
    Vector4f torque_hip_gravity;
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

    Vector4f torque_hip_gravity;
    torque_hip_gravity << -0.86, +0.86, -0.86, +0.86;
    Vector4f pos_hip;
    pos_hip << 0.0, 0.0, 0.0, 0.0;
    Vector4f pos_thigh;
    pos_thigh << 0.67, 0.67, 0.67, 0.67;
    Vector4f pos_calf;
    pos_calf << -1.3, -1.3, -1.3, -1.3;
    Vector12f motor_torque = Vector12f::Zero();
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

void Controller::setTorque(const Vector12f &motor_torque) {
    for (int i = 0; i < 4; i++) {
        lowCmd.motorCmd[3 * i + 0].torque = motor_torque[3 * i + 0] * fmin(fmax(time_ - 2, 0) / 5.0, 1);
        lowCmd.motorCmd[3 * i + 1].torque = motor_torque[3 * i + 1] * fmin(fmax(time_ - 5, 0) / 5.0, 1);
        lowCmd.motorCmd[3 * i + 2].torque = motor_torque[3 * i + 2] * fmin(fmax(time_ - 5, 0) / 5.0, 1);
    }
    for (int i = 0; i < 4; i++) {
        lowCmd.motorCmd[i * 3 + 0].position = PosStopF;
        lowCmd.motorCmd[i * 3 + 0].positionStiffness = 0;
        lowCmd.motorCmd[i * 3 + 0].velocity = 0;
        lowCmd.motorCmd[i * 3 + 0].velocityStiffness = fmin(time_ / 3.0, 1) * (0.1);
        lowCmd.motorCmd[i * 3 + 1].position = PosStopF;
        lowCmd.motorCmd[i * 3 + 1].positionStiffness = 0;
        lowCmd.motorCmd[i * 3 + 1].velocity = 0;
        lowCmd.motorCmd[i * 3 + 1].velocityStiffness = fmin(time_ / 3.0, 1) * (0.04);
        lowCmd.motorCmd[i * 3 + 2].position = PosStopF;
        lowCmd.motorCmd[i * 3 + 2].positionStiffness = 0;
        lowCmd.motorCmd[i * 3 + 2].velocity = 0;
        lowCmd.motorCmd[i * 3 + 2].velocityStiffness = fmin(time_ / 3.0, 1) * (0.02);
    }
}

Matrix3f Controller::conjMatrix(const Vector3f &vec) {
    Matrix3f mat;
    mat << 0, -vec[2], vec[1],
            vec[2], 0, -vec[0],
            -vec[1], vec[0], 0;
    return mat;
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

Vector12f Controller::getKinForceM(const Vector12f &p_feet_desired,
                                                     const Matrix3f &M) {
    Matrix12f M_imu = Matrix12f::Zero();
    for (int i = 0; i < 4; i++) {
        M_imu.block(3 * i, 3 * i, 3, 3) = M;
    }
    Vector12f p_feet_error = p_feet_desired - M_imu * kin_.p_feet_;
    Vector12f feet_force_kin = kp_kin_ * p_feet_error;

    return M_imu.transpose() * feet_force_kin;
}

Vector12f Controller::getKinForce(const Vector12f &p_feet_desired) {
    auto M = Matrix3f::Identity();
    return getKinForceM(p_feet_desired, M);
}

Vector12f Controller::getFpForce(const Vector12f &p_feet_desired) {
    auto R_rp = kin_.R_yaw_.transpose() * kin_.R_imu_;
    static unsigned int s_ptime{0};
    s_ptime = (s_ptime + 1) % sample_t_;
    float k_w = fmin(1, fmax(0, float(s_ptime) / sample_t_));
    return getKinForceM(p_feet_desired, R_rp) * k_w;
}

void Controller::getDynMat(Eigen::Matrix<float, 3, 12> &Mat_lin,
                           Eigen::Matrix<float, 3, 12> &Mat_rot) {
    for (int i = 0; i < 4; i++) {
        Mat_lin.block(0, 3 * i, 3, 3) = Matrix3f::Identity();
        Vector3f foot_world = kin_.R_imu_ * kin_.p_feet_.segment(3 * i, 3);
        Mat_rot.block(0, 3 * i, 3, 3) = kin_.R_yaw_.transpose() * conjMatrix(foot_world);
    }
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

Vector12f Controller::getFpTarget() {
    Vector12f p_feet_desired_fp(p_feet_default_);
    for (int i = 0; i < 4; i++) {
        float k_x{0.0f};
        float k_y{0.2f};
        float height_z{0.1};
        Vector3f p_feet_temp;
        Vector3f p_feet_gain;
        p_feet_temp << vel_x_, vel_y_, height_z;
        p_feet_gain << k_x, k_y, 1;
        Vector3f p_feet_delta;
        Vector3f bodySpeed_offset;
        bodySpeed_offset << 0.01 + kt_[2], 0.005 + kt_[3], 0;
        p_feet_delta = (kin_.R_yaw_.transpose() * p_feet_temp).array()*p_feet_gain.array() + bodySpeed_offset.array();
        p_feet_desired_fp.segment(3 * i, 3) += p_feet_delta;
    }
    return p_feet_desired_fp;
}

void Controller::getAccState(Vector6f &acc_body) {
    Eigen::DiagonalMatrix<float, 6> kp_body, kd_body;
    kp_body.diagonal() << 700 * .0, 700 * .0, 7000, 300, 300, 30;
    kd_body.diagonal() << 10, 30, 0, 0, 0, 1;
    Vector6f desired_pos_body, pos_body, vel_body;
    Vector3f bodySpeed_offset;
    desired_pos_body << 0, 0, 0.4,
            (0 + kp_[0]) * float(M_PI) / 180,
            (0 + kp_[1]) * float(M_PI) / 180,
            (0 + kp_[2]) * float(M_PI) / 180;
    pos_body << est_.worldState_.bodyPosition.x,
            est_.worldState_.bodyPosition.y,
            est_.worldState_.bodyPosition.z,
            kin_.q_imu_;
    vel_body << est_.worldState_.bodySpeed.x,
            est_.worldState_.bodySpeed.y,
            est_.worldState_.bodySpeed.z,
            kin_.dq_imu_;
    bodySpeed_offset << 0.00 + kt_[0], 0.00 + kt_[1], 0;
    vel_body.segment(0, 3) += kin_.R_yaw_ * bodySpeed_offset;
    // todo: add gravity. Note: since the mass is simplify, the g is not 9.8, need to tune it.
    acc_body = kp_body * (desired_pos_body - pos_body) - kd_body * vel_body;
}

const std::vector<Eigen::Vector4i> &Controller::getGaitSet() {
    auto D_stance = Eigen::Vector4i{0, 0, 0, 0};
    auto D_trot1 = Eigen::Vector4i{1, 0, 0, 1};
    auto D_trot2 = Eigen::Vector4i{0, 1, 1, 0};
    auto D_step_FR = Eigen::Vector4i{1, 0, 0, 0};
    auto D_step_FL = Eigen::Vector4i{0, 1, 0, 0};
    auto D_step_RR = Eigen::Vector4i{0, 0, 1, 0};
    auto D_step_RL = Eigen::Vector4i{0, 0, 0, 1};
    static std::vector<Eigen::Vector4i> D_vec{D_stance, D_trot1, D_trot2,
                                                       D_step_FR, D_step_FL, D_step_RR, D_step_RL};
//    static std::vector<Eigen::Vector4i> D_vec{D_stance, D_step_FR, D_step_FL, D_step_RR, D_step_RL};
    return D_vec;
}

int Controller::getGaitIndex(const std::vector<Eigen::Vector4i> &D_vec, const std::vector<float> &cost_vec) {
    Vector12f p_feet_desired_fp = getFpTarget();
    auto R_rp = kin_.R_yaw_.transpose() * kin_.R_imu_;
    std::vector<float> feet_error_sums{};
    std::vector<float> total_cost_vec{};
    float feet_err_weight = 200;
    if (abs(kt_[0]) > 0.02 or abs(kt_[1]) > 0.02) // todo: a better way
        feet_err_weight *= 2;

    const int par_n = D_vec.size();
    for (int j = 0; j < par_n; j++) {
        float sum{0};
        for (int i = 0; i < 4; i++) {
            //todo: consider yaw in error, check the yaw offset, desired yaw and swing foot reference
            // the default should consider swing foot placement
            // might consider use square
            auto p_feet_error = (p_feet_desired_fp.segment(3 * i, 3) -
                                 R_rp * kin_.p_feet_.segment(3 * i, 3)).cwiseAbs();
            if (D_vec[j][i] == 0) {
                sum += p_feet_error(0);
                sum += p_feet_error(1);
            }
        }
        feet_error_sums.push_back(sum);
        total_cost_vec.push_back(sum * feet_err_weight + cost_vec[j]);
    }

    return std::min_element(total_cost_vec.begin(), total_cost_vec.end()) - total_cost_vec.begin();;
}




