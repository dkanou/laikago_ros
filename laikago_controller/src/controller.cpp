#include "../include/controller.h"

Controller::Controller(ros::NodeHandle *n) : n_(*n), multiQp_(getGaitSet()) {
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
    est_.update();
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
    //todo: remove Mat_force and Mat_force_weight
    Eigen::DiagonalMatrix<float, 12> Mat_force_weight;
    getDynMat(Mat_lin, Mat_rot, Mat_force, Mat_force_weight);

    // acceleration
    Eigen::Matrix<float, 6, 1> acc_body;
    Eigen::Matrix<float, 12, 1> acc_feet;
    getAccState(acc_body, acc_feet);

    // linear optimization
    Eigen::Matrix<float, 6, 12> opt_A;
    Eigen::Matrix<float, 6, 1> opt_b;
    opt_A << Mat_lin, Mat_rot;
    opt_A.block(0, 0, 2, 12) *= 1e-1;   //todo: find another way to adjust
    opt_b = acc_body;

    // feet pattern
    const auto D_vec = getGaitSet();
    const int par_n = D_vec.size();
    multiQp_.multiSolve(opt_A, opt_b, grf_index_);
    const auto grf_qp_vec = multiQp_.getGrf();
    const auto cost_vec = multiQp_.getCost();

    int min_cost_index = getGaitIndex(D_vec, cost_vec);

    static unsigned int s_ptime{0};
    ++s_ptime;
    if (s_ptime%175==0){ // 175*0.002 = 0.35s(~3Hz)
        grf_index_ = min_cost_index;
    }

    // swap legs
    auto grf_in = grf_qp_vec[grf_index_];
    for (int i = 0; i < 4; i++) {
        if (D_vec[grf_index_][i] == 1) {
            grf_in.segment(3*i, 3) = acc_feet.segment(3*i, 3);
        }
    }
    Eigen::Matrix<float, 12, 1> grf_qp;
    float fil_const = 0.1;
    for (int i=0; i<12; i++){
        grf_qp(i) = lowPassFilter_[i].firstOrder(grf_in(i), fil_const);
    }

    // grf to motor force
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


Eigen::Matrix<float, 12, 1> Controller::getFpTarget() {
    Eigen::Matrix<float, 12, 1> p_feet_desired_fp(p_feet_default_);
    for (int i = 0; i < 4; i++) {
        float k_x{0.1/2};   //todo: temporarily decrease for gait switch control
        float k_y{0.2/2};
        float height_z{0.08};
        Eigen::Matrix<float, 3, 1> p_feet_delta;
        //todo: the velocity is too noisy. Re-tune the filter weight
        float vel_x{lowPassFilterVel_[0].firstOrder(est_.worldState_.bodySpeed.x, 0.01)};
        float vel_y{lowPassFilterVel_[1].firstOrder(est_.worldState_.bodySpeed.y, 0.01)};
//        printf("vel y = %f, est vel y = %f\n", est_.worldState_.bodySpeed.y, vel_y);
        p_feet_delta << vel_x * (k_x + kd_[0]),
                vel_y * (k_y + kd_[1]),
                height_z;
        p_feet_desired_fp.segment(3 * i, 3) += kin_.R_yaw_.transpose() * p_feet_delta;
    }

    return p_feet_desired_fp;
}


void Controller::getAccState(Eigen::Matrix<float, 6, 1> &acc_body, Eigen::Matrix<float, 12, 1> &acc_feet) {
    Eigen::DiagonalMatrix<float, 6> kp_body, kd_body;
    kp_body.diagonal() << 700 * .0, 700 * .0, 7000, 300, 300, 300;
    kd_body.diagonal() << 30, 30, 0, 0, 0, 0.5;
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
    bodySpeed_offset << 0.0+kt_[0], 0.04+kt_[1], 0;
    vel_body.segment(0, 3) += kin_.R_yaw_ * bodySpeed_offset;
    acc_body = fmin(time_ / 10.0, 1) * (kp_body * (desired_pos_body - pos_body) - kd_body * vel_body);

    // foot placement
    Eigen::Matrix<float, 12, 1> p_feet_desired_fp = getFpTarget();

    //todo: the desired feet should be in world frame not in body frame
    float k_w{0.5}; //todo: temporarily decrease for switching gait control
    auto feet_force_fp = getKinForce(p_feet_desired_fp) * k_w;
    for (int i = 0; i < 4; i++) {
        // attention: the grf is opposite of the motor force
        acc_feet.segment(3 * i, 3) = kin_.R_imu_ * -feet_force_fp.segment(3 * i, 3);
    }
}

const std::vector<Eigen::Matrix<int, 4, 1>> &Controller::getGaitSet() {
    auto D_stance = Eigen::Matrix<int, 4, 1>{0, 0, 0, 0};
    auto D_trot1 = Eigen::Matrix<int, 4, 1>{1, 0, 0, 1};
    auto D_trot2 = Eigen::Matrix<int, 4, 1>{0, 1, 1, 0};
    auto D_step_FR = Eigen::Matrix<int, 4, 1>{1, 0, 0, 0};
    auto D_step_FL = Eigen::Matrix<int, 4, 1>{0, 1, 0, 0};
    auto D_step_RR = Eigen::Matrix<int, 4, 1>{0, 0, 1, 0};
    auto D_step_RL = Eigen::Matrix<int, 4, 1>{0, 0, 0, 1};
    static std::vector<Eigen::Matrix<int, 4, 1>> D_vec{D_stance, D_trot1, D_trot2,
                                                  D_step_FR, D_step_FL, D_step_RR, D_step_RL};
//    static std::vector<Eigen::Matrix<int, 4, 1>> D_vec{D_stance, D_step_FR, D_step_FL, D_step_RR, D_step_RL};
    return D_vec;
}

int Controller::getGaitIndex(const std::vector<Eigen::Matrix<int, 4, 1>> &D_vec, const std::vector<float> &cost_vec) {
    Eigen::Matrix<float, 12, 1> p_feet_desired_fp = getFpTarget();
    Eigen::Matrix<float, 3, 3> R_yaw_offset;
    R_yaw_offset << cos(yaw_offset_), -sin(yaw_offset_), 0,
            sin(yaw_offset_), cos(yaw_offset_), 0,
            0, 0, 1;
    std::vector<float> feet_error_sums{};
    std::vector<float> total_cost_vec{};
    float feet_err_weight = 300;
    const int par_n = D_vec.size();
    for (int j=0; j<par_n; j++) {
        float sum{0};
        for (int i = 0; i < 4; i++) {
            //todo: consider yaw in error, check the yaw offset, desired yaw and swing foot reference
            // the default should consider swing foot placement
            // might consider use square
            auto p_feet_error = (p_feet_desired_fp.segment(3 * i, 3) -
                                 R_yaw_offset.transpose() * kin_.R_imu_ * kin_.p_feet_.segment(3 * i, 3)).cwiseAbs();
            if (D_vec[j][i] == 0) {
                sum += p_feet_error(0);
                sum += p_feet_error(1);
            }
        }
        feet_error_sums.push_back(sum);
        total_cost_vec.push_back(sum*feet_err_weight + cost_vec[j]);
    }

    return std::min_element(total_cost_vec.begin(),total_cost_vec.end()) - total_cost_vec.begin();;
}




