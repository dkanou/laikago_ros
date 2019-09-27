#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <laikago_msgs/HighState.h>
#include "laikago_sdk/laikago_sdk.hpp"

namespace robot_hal {

class BodyPoseEstimator {
public:
    BodyPoseEstimator() {
        _xhat.setZero();
        _xhat(2) = 1; //height initialization
        _ps.setZero();
        _vs.setZero();
        _A.setZero();
        _A.block(0, 0, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
        _A.block(0, 3, 3, 3) = dt_ * Eigen::Matrix<float, 3, 3>::Identity();
        _A.block(3, 3, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
        _A.block(6, 6, 12, 12) = Eigen::Matrix<float, 12, 12>::Identity();
        _B.setZero();
        _B.block(3, 0, 3, 3) = dt_ * Eigen::Matrix<float, 3, 3>::Identity();
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
        C1 << Eigen::Matrix<float, 3, 3>::Identity(), Eigen::Matrix<float, 3, 3>::Zero();
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
        C2 << Eigen::Matrix<float, 3, 3>::Zero(), Eigen::Matrix<float, 3, 3>::Identity();
        _C.setZero();
        _C.block(0, 0, 3, 6) = C1;
        _C.block(3, 0, 3, 6) = C1;
        _C.block(6, 0, 3, 6) = C1;
        _C.block(9, 0, 3, 6) = C1;
        _C.block(0, 6, 12, 12) = float(-1) * Eigen::Matrix<float, 12, 12>::Identity();
        _C.block(12, 0, 3, 6) = C2;
        _C.block(15, 0, 3, 6) = C2;
        _C.block(18, 0, 3, 6) = C2;
        _C.block(21, 0, 3, 6) = C2;
        _C(27, 17) = float(1);
        _C(26, 14) = float(1);
        _C(25, 11) = float(1);
        _C(24, 8) = float(1);
        _P.setIdentity();
        _P = float(100) * _P;
        _Q0.setIdentity();
        _Q0.block(0, 0, 3, 3) = (dt_ / 20.f) * Eigen::Matrix<float, 3, 3>::Identity();
        _Q0.block(3, 3, 3, 3) =
                (dt_ * 9.8f / 20.f) * Eigen::Matrix<float, 3, 3>::Identity();
        _Q0.block(6, 6, 12, 12) = dt_ * Eigen::Matrix<float, 12, 12>::Identity();
        _R0.setIdentity();
    }

    void update(const laikago_msgs::HighState &state);

    Eigen::Vector3f getBodyPosition() const {
        return _xhat.block(0, 0, 3, 1);
    }

    Eigen::Vector3f getBodyVelocity() const {
        return _xhat.block(3, 0, 3, 1);
    }

private:
    const float dt_{0.002}; //todo(dda): need to agree with the sample rate
    Eigen::Matrix<float, 18, 1> _xhat;
    Eigen::Matrix<float, 12, 1> _ps;
    Eigen::Matrix<float, 12, 1> _vs;
    Eigen::Matrix<float, 18, 18> _A;
    Eigen::Matrix<float, 18, 18> _Q0;
    Eigen::Matrix<float, 18, 18> _P;
    Eigen::Matrix<float, 28, 28> _R0;
    Eigen::Matrix<float, 18, 3> _B;
    Eigen::Matrix<float, 28, 18> _C;
    const float low_{15};
    const float high_{60};
};


void BodyPoseEstimator::update(const laikago_msgs::HighState &state) {

    float process_noise_pimu = 0.02;
    float process_noise_vimu = 0.02;
    float process_noise_pfoot = 0.002;
    float sensor_noise_pimu_rel_foot = 0.001;
    float sensor_noise_vimu_rel_foot = 0.1;
    float sensor_noise_zfoot = 0.001;

    Eigen::Matrix<float, 18, 18> Q = Eigen::Matrix<float, 18, 18>::Identity();
    Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
    Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
    Q.block(6, 6, 12, 12) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot;

    Eigen::Matrix<float, 28, 28> R = Eigen::Matrix<float, 28, 28>::Identity();
    R.block(0, 0, 12, 12) = _R0.block(0, 0, 12, 12) * sensor_noise_pimu_rel_foot;
    R.block(12, 12, 12, 12) =
            _R0.block(12, 12, 12, 12) * sensor_noise_vimu_rel_foot;
    R.block(24, 24, 4, 4) = _R0.block(24, 24, 4, 4) * sensor_noise_zfoot;

    int qindex = 0;
    int rindex1 = 0;
    int rindex2 = 0;
    int rindex3 = 0;

    Eigen::Quaternionf imu_quat(state.imu.quaternion.elems[0],
                                state.imu.quaternion.elems[1],
                                state.imu.quaternion.elems[2],
                                state.imu.quaternion.elems[3]);
    Eigen::Matrix3f Rbod = imu_quat.toRotationMatrix();
    Eigen::Vector3f a;
    a << state.imu.acceleration.elems[0],
            state.imu.acceleration.elems[1],
            state.imu.acceleration.elems[2]; // with gravity bias
    a = Rbod * a + Eigen::Vector3f{0.0f, 0.0f, -9.81f};
    Eigen::Vector4f pzs = Eigen::Vector4f::Zero();
    Eigen::Vector4f trusts = Eigen::Vector4f::Zero();
    Eigen::Vector3f p0, v0;
    p0 << _xhat[0], _xhat[1], _xhat[2];
    v0 << _xhat[3], _xhat[4], _xhat[5];

    for (int i = 0; i < 4; i++) {
        int i1 = 3 * i;
        Eigen::Vector3f p_rel;
        p_rel << state.footPosition2Body.elems[i].x,
                state.footPosition2Body.elems[i].y,
                state.footPosition2Body.elems[i].z;
        Eigen::Vector3f dp_rel;
        dp_rel << state.footSpeed2Body.elems[i].x,
                state.footSpeed2Body.elems[i].y,
                state.footSpeed2Body.elems[i].z;
        Eigen::Vector3f p_f = Rbod * p_rel;
        Eigen::Vector3f dp_f = Rbod * dp_rel;

        qindex = 6 + i1;
        rindex1 = i1;
        rindex2 = 12 + i1;
        rindex3 = 24 + i;
        float phase = fminf(fmaxf((state.footForce.elems[i] - low_) / (high_ - low_), 0), 1);
        float trust = phase; // approach is different from mit cheetah

        // printf("Trust %d: %.3f\n", i, trust);
        Q.block(qindex, qindex, 3, 3) =
                (1 + (1 - trust) * 100) * Q.block(qindex, qindex, 3, 3);
        R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
        R.block(rindex2, rindex2, 3, 3) =
                (1 + (1 - trust) * 100.0f) * R.block(rindex2, rindex2, 3, 3);
        R(rindex3, rindex3) =
                (1 + (1 - trust) * 100) * R(rindex3, rindex3);

        trusts(i) = trust;

        _ps.segment(i1, 3) = -p_f;
        _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
        pzs(i) = (1.0f - trust) * (p0(2) + p_f(2)) + trust * 0;
    }

    Eigen::Matrix<float, 28, 1> y;
    y << _ps, _vs, pzs;
    _xhat = _A * _xhat + _B * a;
    Eigen::Matrix<float, 18, 18> At = _A.transpose();
    Eigen::Matrix<float, 18, 18> Pm = _A * _P * At + Q;
    Eigen::Matrix<float, 18, 28> Ct = _C.transpose();
    Eigen::Matrix<float, 28, 1> yModel = _C * _xhat;
    Eigen::Matrix<float, 28, 1> ey = y - yModel;
    Eigen::Matrix<float, 28, 28> S = _C * Pm * Ct + R;

    // todo compute LU only once
    Eigen::Matrix<float, 28, 1> S_ey = S.lu().solve(ey);
    _xhat += Pm * Ct * S_ey;

    Eigen::Matrix<float, 28, 18> S_C = S.lu().solve(_C);
    _P = (Eigen::Matrix<float, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;

    Eigen::Matrix<float, 18, 18> Pt = _P.transpose();
    _P = (_P + Pt) / float(2);

    if (_P.block(0, 0, 2, 2).determinant() > float(0.000001)) {
        _P.block(0, 2, 2, 16).setZero();
        _P.block(2, 0, 16, 2).setZero();
        _P.block(0, 0, 2, 2) /= float(10);
    }

}

}  // namespace robot_hal
