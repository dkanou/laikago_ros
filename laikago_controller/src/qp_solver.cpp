#include "../include/qp_solver.h"
#include <ros/ros.h>

QpSolver::QpSolver() : opti_("conic") {
    x_ = opti_.variable(12, 1);
    A_ = opti_.parameter(18, 12);
    b_ = opti_.parameter(18, 1);
    opti_.minimize(dot(mtimes(A_, x_) - b_, mtimes(A_, x_) - b_));

    double mu = 0.3;
    for (int i = 0; i < 4; i++) {
        opti_.subject_to(x_(3 * i + 2) >= 0);
        opti_.subject_to(x_(3 * i + 0) <= mu * x_(3 * i + 2));
        opti_.subject_to(x_(3 * i + 1) <= mu * x_(3 * i + 2));
        opti_.subject_to(-x_(3 * i + 0) <= mu * x_(3 * i + 2));
        opti_.subject_to(-x_(3 * i + 1) <= mu * x_(3 * i + 2));
    }

    Dict opts;
    opts["print_time"] = false;
    // qpoases options
    opts["printLevel"] = "none";
    opti_.solver("qpoases", opts);

    // osqp options.
    // TODO: check the accuracy
//    Dict opts_osqp;
//    opts_osqp["verbose"] = false;
//    opts["osqp"] = opts_osqp;
//    opti_.solver("osqp", opts);

    opti_f_ = opti_.to_function("F", {A_, b_}, {x_});
}

Eigen::Matrix<float, 12, 1> QpSolver::solve(const Eigen::MatrixXf &A, const Eigen::MatrixXf &b) {
    DM dm_A(Sparsity::dense(A.rows(), A.cols()),
            std::vector<double>(A.data(), A.data() + A.size()));
    DM dm_b(Sparsity::dense(b.rows(), b.cols()),
            std::vector<double>(b.data(), b.data() + b.size()));

    // function call (fast)
    std::vector<DM> sol = opti_f_(std::vector<DM>{dm_A, dm_b});
    std::vector<float> vector_x(sol[0].nonzeros().begin(), sol[0].nonzeros().end());
    Eigen::Matrix<float, 12, 1> res_x(vector_x.data());

    return res_x;
}

QpLow::QpLow() : A_(4 * 5, 12) {
    double mu = 0.3;
    for (int i = 0; i < 4; i++) {
        A_(5 * i + 0, 3 * i + 2) = -1;
        A_(5 * i + 1, 3 * i + 0) = 1;
        A_(5 * i + 1, 3 * i + 2) = -mu;
        A_(5 * i + 2, 3 * i + 1) = 1;
        A_(5 * i + 2, 3 * i + 2) = -mu;
        A_(5 * i + 3, 3 * i + 0) = -1;
        A_(5 * i + 3, 3 * i + 2) = -mu;
        A_(5 * i + 4, 3 * i + 1) = -1;
        A_(5 * i + 4, 3 * i + 2) = -mu;
    }
    SpDict qp;
    qp["h"] = Sparsity::dense(12, 12);
    qp["a"] = A_.sparsity();
    Dict opts;
    opts["print_time"] = false;
    opts["printLevel"] = "none";

    S_ = conic("S", "qpoases", qp, opts);
}

Eigen::Matrix<float, 12, 1> QpLow::solve(const Eigen::MatrixXf &A_in, const Eigen::MatrixXf &b_in) {
    Eigen::MatrixXf eig_H = A_in.transpose() * A_in;
    Eigen::MatrixXf eig_g = -A_in.transpose() * b_in;
    DM H(eig_H.rows(), eig_H.cols());
    for (int i = 0; i < H.rows(); i++) {
        for (int j = 0; j < H.columns(); j++) {
            H(i, j) = eig_H(i, j);
        }
    }
    DM g(eig_g.rows(), eig_g.cols());
    for (int i = 0; i < g.rows(); i++) {
        g(i, 0) = eig_g(i, 0);
    }
    DMDict arg = {{"h",   H},
                  {"g",   g},
                  {"a",   A_},
                  {"uba", DM::zeros(4 * 5)}};
    DMDict r = S_(arg);
    
    auto vector_x = static_cast<std::vector<float>>(r["x"]);
    return Eigen::Matrix<float, 12, 1>(vector_x.data());
}
