#include "../include/qp_solver.h"
#include <omp.h>

QpProblem::QpProblem() {
    Opti opti("conic");
    auto u = opti.variable(12, 1);
    auto A = opti.parameter(6, 12);
    auto b = opti.parameter(6, 1);
    double R = 1e-3;
    auto cost = dot(mtimes(A, u) - b, mtimes(A, u) - b) + R * dot(u, u);
    opti.minimize(cost);

    // friction cone
    double mu = 0.6;
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

    opti_f_ = opti.to_function("F", {A, b, D}, {u, cost});
    opti_par_f_ = opti_f_.map(7, "thread", 12);
}

std::pair<Vector12f , float> QpProblem::solve(const Eigen::MatrixXf &A,
                                                               const Eigen::MatrixXf &b,
                                                               const Eigen::MatrixXi &D) {
    DM dm_A(Sparsity::dense(A.rows(), A.cols()),
            std::vector<double>(A.data(), A.data() + A.size()));
    DM dm_b(Sparsity::dense(b.rows(), b.cols()),
            std::vector<double>(b.data(), b.data() + b.size()));
    DM dm_D(Sparsity::dense(D.rows(), D.cols()),
            std::vector<double>(D.data(), D.data() + D.size()));

    std::vector<DM> sol = opti_f_(std::vector<DM>{dm_A, dm_b, dm_D});

    std::vector<float> vector_u(sol[0].nonzeros().begin(), sol[0].nonzeros().end());
    Vector12f u(vector_u.data());
    float cost = sol[1].nonzeros().at(0);
    return {u, cost};
}

MultiQp::MultiQp(const std::vector<Eigen::Vector4i> &D_vec) :
        D_vec_(D_vec), par_n_(D_vec.size()) {
    grf_qp_vec_ = std::vector<Vector12f>(par_n_, Vector12f::Zero());
    cost_vec_ = std::vector<float>(par_n_, 0);
}

void MultiQp::multiSolve(const Eigen::MatrixXf &A,
                const Eigen::MatrixXf &b,
                unsigned int curr_index) {
    static unsigned int count{0};
    unsigned int roll_index = count%par_n_;
    for (auto i : {curr_index, roll_index} ) {
        auto res = qpProblem_[i].solve(A, b, D_vec_[i]);
        grf_qp_vec_[i] = res.first;
        cost_vec_[i] = res.second;
    }
    ++count;
}