#include "../include/qp_solver.h"

QpSolver::QpSolver(): opti_("conic") {
    x_ = opti_.variable(12, 1);
    A_ = opti_.parameter(18, 12);
    b_ = opti_.parameter(18, 1);
    opti_.minimize(dot(mtimes(A_, x_) - b_, mtimes(A_, x_) - b_));

    double mu = 0.4;
    for (int i = 0; i < 4; i++) {
        opti_.subject_to(x_(3 * i + 2) <= 0);
        opti_.subject_to(fabs(x_(3 * i + 0)) <= -mu * x_(3 * i + 2));
        opti_.subject_to(fabs(x_(3 * i + 1)) <= -mu * x_(3 * i + 2));
    }

    Dict opts;
    opts["print_time"] = false;
    opts["printLevel"] = "none";
    opti_.solver("qpoases", opts);
}

Eigen::Matrix<float, 12, 1> QpSolver::solve(Eigen::MatrixXf &A, Eigen::MatrixXf &b) {
    DM dm_A(std::vector<double>(A.data(), A.data() + A.size()));
    dm_A = reshape(dm_A, A_.rows(), A_.columns());
    DM dm_b(std::vector<double>(b.data(), b.data() + b.size()));
    dm_b = reshape(dm_b, b_.rows(), b_.columns());

    opti_.set_value(A_, dm_A);
    opti_.set_value(b_, dm_b);

    auto sol = opti_.solve();
    Eigen::Map<Eigen::VectorXd> res_x(sol.value(x_).ptr(), x_.rows());
    return res_x.cast<float>();
}

