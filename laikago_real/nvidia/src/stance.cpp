#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <laikago_msgs/LowCmd.h>
#include <laikago_msgs/LowState.h>
#include "laikago_sdk/laikago_sdk.hpp"
#include "../../../laikago_controller/include/controller.h"

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

void* update_loop(void* data)
{
    while(ros::ok){
        boost::mutex::scoped_lock lock(mutex);
        roslcm.Recv();
        lock.unlock();
        usleep(2000);
    }
}

int main(int argc, char *argv[])
{
    system("echo '' | sudo -S ${Laikago_SDK}/cmake-build-debug/sdk_lcm_server_low &");
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl;

    ros::init(argc, argv, "neutral_mode_low");
    ros::NodeHandle n;
    ros::Rate loop_rate(500);
    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop, NULL);

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 1; i<13; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
    }

    while (ros::ok()){
        motiontime++;
        roslcm.Get(RecvLowLCM);
        memcpy(&RecvLowROS, &RecvLowLCM, sizeof(LowState));
        memcpy(&SendLowLCM, &SendLowROS, sizeof(LowCmd));
        roslcm.Send(SendLowLCM);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
