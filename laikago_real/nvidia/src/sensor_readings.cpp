#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <Eigen/Geometry>
#include <laikago_msgs/HighCmd.h>
#include <laikago_msgs/HighState.h>
#include <pthread.h>
#include <ros/ros.h>
#include "laikago_sdk/laikago_sdk.hpp"
#include "state_estimation.h"

using namespace laikago;

static long motiontime = 0;
HighCmd SendHighLCM = {0};
HighState RecvHighLCM = {0};
laikago_msgs::HighCmd SendHighROS;
laikago_msgs::HighState RecvHighROS;

Control control(HIGHLEVEL);
LCM roslcm;
boost::mutex mutex;

void *update_loop(void *data) {
    while (ros::ok) {
        boost::mutex::scoped_lock lock(mutex);
        roslcm.Recv();
        lock.unlock();
        usleep(2000);
    }
}

int main(int argc, char *argv[]) {
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl;

    ros::init(argc, argv, "sensor_readings");
    ros::NodeHandle n;
    ros::Rate loop_rate(500);
    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop, NULL);

    // init control state
    robot_hal::BodyPoseEstimator bodyPoseEstimator;

    while (ros::ok()) {
        motiontime = motiontime + 2;
        roslcm.Get(RecvHighLCM);
        memcpy(&RecvHighROS, &RecvHighLCM, sizeof(HighState));

        // parse the state
        Eigen::Quaternionf imu_quat(RecvHighROS.imu.quaternion.elems[0],
                                    RecvHighROS.imu.quaternion.elems[1],
                                    RecvHighROS.imu.quaternion.elems[2],
                                    RecvHighROS.imu.quaternion.elems[3]);
        Eigen::Matrix3f mat_R = imu_quat.toRotationMatrix();

        bodyPoseEstimator.update(RecvHighROS);

        // pass command
        memcpy(&SendHighLCM, &SendHighROS, sizeof(HighCmd));
        roslcm.Send(SendHighLCM);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

