#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include "../../../laikago_controller/include/qp_solver.h"

using namespace casadi;

int main() {
    Eigen::MatrixXf A = Eigen::MatrixXf::Identity(18, 12);
    Eigen::MatrixXf b = Eigen::MatrixXf::Zero(18, 1);
    QpSolver qp_solver;
    for (int i = 0; i < 4; i++) {
        Eigen::Matrix<float, 12, 1> res_x = qp_solver.solve(A, b);
        std::cout << i << " " << res_x.transpose() << std::endl;
    }

    return 0;
}
