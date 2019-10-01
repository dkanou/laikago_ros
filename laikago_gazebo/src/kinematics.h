#ifndef CATKIN_WS_KINEMATICS_H
#define CATKIN_WS_KINEMATICS_H

#include "body.h"
#include "ros/ros.h"

using laikago_model::lowState;

namespace laikago {
class Kinematics {
public:
    void readSensors() {

    }

    static void  __attribute__ ((used)) printState() {
        std::cout << lowState << std::endl;
    }
};

}


#endif //CATKIN_WS_KINEMATICS_H
