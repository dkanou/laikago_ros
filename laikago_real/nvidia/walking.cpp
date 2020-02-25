#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <laikago_msgs/LowCmd.h>
#include <laikago_msgs/LowState.h>
#include "laikago_sdk/laikago_sdk.hpp"
#include "controller.h"

using namespace laikago;

static long motiontime = 0;
float Kv[3] = {0};
float Kp[3] = {0};
float movejoint1 = 0, movejoint2 = 0;
unsigned long int Tpi = 0;
LowCmd SendLowLCM = {0};
LowState RecvLowLCM = {0};
laikago_msgs::LowCmd SendLowROS;
laikago_msgs::LowState RecvLowROS;

Control control(LOWLEVEL);
LCM roslcm;
boost::mutex mutex;

void *update_loop(void *data) {
    while (ros::ok()) {
        boost::mutex::scoped_lock lock(mutex);
        roslcm.Recv();
        lock.unlock();
        usleep(2000);
    }
}

//todo: a better way to declare sim or real
bool Kinematics::sim = false;

int main(int argc, char *argv[]) {
    //todo: do not use system and sudo
    if (system("sudo ${Laikago_SDK}/cmake-build-debug/sdk_lcm_server_low &") != 0) { return -1; };
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl;

    ros::init(argc, argv, "walking");
    ros::NodeHandle n;
    ros::Rate loop_rate(500);
    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, nullptr, update_loop, nullptr);

    ros::Publisher lowState_pub = n.advertise<laikago_msgs::LowState>("/laikago_gazebo/lowState/state", 1);
    ros::Publisher highState_pub = n.advertise<laikago_msgs::HighState>("/laikago_gazebo/highState/state", 1);

    SendLowROS.levelFlag = LOWLEVEL;
    for (int i = 1; i < 13; i++) {
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
    }

    Controller controller(&n);
    double begin_time = ros::Time::now().toSec();

    while (ros::ok()) {
        motiontime++;
        roslcm.Get(RecvLowLCM);
        memcpy(&RecvLowROS, &RecvLowLCM, sizeof(RecvLowROS));

        // control algorithm
        Kinematics::setLowState(RecvLowROS);
        double sim_time = ros::Time::now().toSec() - begin_time;
        controller.setTime(sim_time);
        controller.sendCommand();
        Kinematics::setLowCmd(SendLowROS);

        // publish state and cmd
        lowState_pub.publish(lowState);
        highState_pub.publish(highState);
        memcpy(&SendLowLCM, &SendLowROS, sizeof(SendLowLCM));
        roslcm.Send(SendLowLCM);

        // finish one iteration
        ros::spinOnce();
//        printf("cycle time = %f\n", loop_rate.cycleTime().toSec());
        loop_rate.sleep();
    }
    return 0;
}
