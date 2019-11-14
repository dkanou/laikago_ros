#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include "../../../laikago_controller/include/qp_solver.h"

using namespace casadi;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "test_casadi");
    ros::NodeHandle n;
    Eigen::MatrixXf A = Eigen::MatrixXf::Identity(18, 12);
    Eigen::MatrixXf b = Eigen::MatrixXf::Zero(18, 1);
    b(0) = 1;
    QpSolver qp_solver;
    Eigen::Matrix<float, 12, 1> res_x;
    for (int i = 0; i <1e3; i++) {
        res_x = qp_solver.solve(A, b);
//    std::cout << 0 << " " << res_x.transpose() << std::endl;
    }

    QpLow qp_low;
    for (int i = 0; i <1e3; i++){
        res_x = qp_low.solve(A, b);
//        std::cout << 0 << " " << res_x.transpose() << std::endl;
    }

    return 0;
}
