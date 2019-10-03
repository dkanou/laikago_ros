#ifndef CATKIN_WS_CONTROLLER_H
#define CATKIN_WS_CONTROLLER_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include "body.h"
#include "kinematics.h"

using namespace casadi;
using laikago_model::lowCmd;

class Controller {
public:
    void sendCommand() {
        kin_.update();
        setMotorZero();
    }

    void setTime(const double &time) {
        time_ = time;
    }

    static void setMotorZero() {
        for(int i=0; i<4; i++){
            lowCmd.motorCmd[i*3+0].mode = 0x0A;
            lowCmd.motorCmd[i*3+0].position = PosStopF;
            lowCmd.motorCmd[i*3+0].positionStiffness = 0;
            lowCmd.motorCmd[i*3+0].velocity = VelStopF;
            lowCmd.motorCmd[i*3+0].velocityStiffness = 0;
            lowCmd.motorCmd[i*3+0].torque = 0;
            lowCmd.motorCmd[i*3+1].mode = 0x0A;
            lowCmd.motorCmd[i*3+1].position = PosStopF;
            lowCmd.motorCmd[i*3+1].positionStiffness = 0;
            lowCmd.motorCmd[i*3+1].velocity = VelStopF;
            lowCmd.motorCmd[i*3+1].velocityStiffness = 0;
            lowCmd.motorCmd[i*3+1].torque = 0;
            lowCmd.motorCmd[i*3+2].mode = 0x0A;
            lowCmd.motorCmd[i*3+2].position = PosStopF;
            lowCmd.motorCmd[i*3+2].positionStiffness = 0;
            lowCmd.motorCmd[i*3+2].velocity = VelStopF;
            lowCmd.motorCmd[i*3+2].velocityStiffness = 0;
            lowCmd.motorCmd[i*3+2].torque = 0;
        }
    }

private:
    double time_{0};
    Kinematics kin_;
};


#endif //CATKIN_WS_CONTROLLER_H
