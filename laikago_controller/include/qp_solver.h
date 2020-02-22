#ifndef CATKIN_WS_QP_SOLVER_H
#define CATKIN_WS_QP_SOLVER_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>

using namespace casadi;

class QpProblem {
public:
    QpProblem();

    Eigen::Matrix<float, 12, 1> solve(const Eigen::MatrixXf &A,
                                      const Eigen::MatrixXf &b,
                                      const Eigen::MatrixXf &D,
                                      float &lost);

private:
    Function opti_f_;
};

#endif //CATKIN_WS_QP_SOLVER_H
