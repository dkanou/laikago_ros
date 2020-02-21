#include "../include/qp_solver.h"

QpProblem::QpProblem() {
    Opti opti("conic");
    auto u = opti.variable(12, 1);
    auto A = opti.parameter(6, 12);
    auto b = opti.parameter(6, 1);
    double R = 1e-3;
    auto lost = dot(mtimes(A, u) - b, mtimes(A, u) - b) + R * dot(u, u);
    opti.minimize(lost);

    // friction cone
    double mu = 0.3;
    for (int i = 0; i < 4; i++) {
        opti.subject_to(u(3 * i + 2) >= 0);
        opti.subject_to(u(3 * i + 0) <= mu * u(3 * i + 2));
        opti.subject_to(u(3 * i + 1) <= mu * u(3 * i + 2));
        opti.subject_to(-u(3 * i + 0) <= mu * u(3 * i + 2));
        opti.subject_to(-u(3 * i + 1) <= mu * u(3 * i + 2));
    }
    // contact constraint
    auto D = opti.parameter(4, 1);
    for (int i = 0; i < 4; i++) {
        opti.subject_to(D(i) * u(3 * i + 2) == 0);
    }
    Dict opts;
    opts["print_time"] = false;
    opts["printLevel"] = "none";
    opti.solver("qpoases", opts);

    opti_f_ = opti.to_function("F", {A, b, D}, {u});
}

Eigen::Matrix<float, 12, 1> QpProblem::solve(const Eigen::MatrixXf &A,
                                  const Eigen::MatrixXf &b,
                                  const Eigen::MatrixXf &D) {
    DM dm_A(Sparsity::dense(A.rows(), A.cols()),
            std::vector<double>(A.data(), A.data() + A.size()));
    DM dm_b(Sparsity::dense(b.rows(), b.cols()),
            std::vector<double>(b.data(), b.data() + b.size()));
    DM dm_D(Sparsity::dense(D.rows(), D.cols()),
            std::vector<double>(D.data(), D.data() + D.size()));

    std::vector<DM> sol = opti_f_(std::vector<DM>{dm_A, dm_b, dm_D});
    std::vector<float> vector_u(sol[0].nonzeros().begin(), sol[0].nonzeros().end());
    Eigen::Matrix<float, 12, 1> res_u(vector_u.data());
    return res_u;
}