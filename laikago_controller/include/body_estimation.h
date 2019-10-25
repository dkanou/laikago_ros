#ifndef CATKIN_WS_BODY_ESTIMATION_H
#define CATKIN_WS_BODY_ESTIMATION_H

#include <Eigen/Dense>
#include "body.h"
#include "laikago_msgs/WorldState.h"

using laikago_model::lowState;
using laikago_model::highState;

class BodyPoseEstimator {
public:
    BodyPoseEstimator();
    void update();
    void publish() { worldState_pub_.publish(worldState_); }

    laikago_msgs::WorldState worldState_;

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
    const float low_{10};
    const float high_{20};
    ros::Publisher worldState_pub_;
};



#endif //CATKIN_WS_BODY_ESTIMATION_H
