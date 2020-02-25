#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include "qp_solver.h"

using namespace casadi;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "test_casadi");
    ros::NodeHandle n;
    Eigen::MatrixXf A = Eigen::MatrixXf::Identity(6, 12);
    Eigen::MatrixXf b = Eigen::MatrixXf::Ones(6, 1);
//    Eigen::MatrixXf D = Eigen::MatrixXf::Ones(12, 1);
    Eigen::MatrixXf D = Eigen::MatrixXf::Zero(4, 1);
    QpProblem qpProblem;
    auto res = qpProblem.solve(A, b, D);
    auto u = res.first;
    auto cost = res.second;
    std::cout << u.transpose() << " " << cost << std::endl;

    return 0;
}
