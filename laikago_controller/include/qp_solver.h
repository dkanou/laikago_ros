#ifndef CATKIN_WS_QP_SOLVER_H
#define CATKIN_WS_QP_SOLVER_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include "eigen_def.h"

using namespace casadi;

class QpProblem {
public:
    QpProblem();

    std::pair<Vector12f, float> solve(const Eigen::MatrixXf &A,
                                                        const Eigen::MatrixXf &b,
                                                        const Eigen::MatrixXi &D);

private:
    Function opti_f_;
    Function opti_par_f_;
};

//todo: refactor the D_vec and the initialization process
class MultiQp {
public:
    MultiQp(const std::vector<Eigen::Vector4i> &D_vec);

    void multiSolve(const Eigen::MatrixXf &A,
                    const Eigen::MatrixXf &b,
                    unsigned int curr_index);

    const std::vector<Vector12f> &getGrf() { return grf_qp_vec_; }

    const std::vector<float> &getCost() { return cost_vec_; }

private:
    const std::vector<Eigen::Vector4i> &D_vec_;
    const int par_n_;
    std::vector<Vector12f> grf_qp_vec_;
    std::vector<float> cost_vec_;
    QpProblem qpProblem_[16]; //todo: match par_n_
};

#endif //CATKIN_WS_QP_SOLVER_H
