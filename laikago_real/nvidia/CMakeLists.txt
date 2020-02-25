find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)

include_directories(../../laikago_controller/include)

# Declare a C++ library
# TODO simplify the path
add_library(control_lib
    ../../laikago_controller/src/body.cpp
    ../../laikago_controller/src/kinematics.cpp
    ../../laikago_controller/src/body_estimation.cpp
    ../../laikago_controller/src/qp_solver.cpp
    ../../laikago_controller/src/controller.cpp
    )

target_link_libraries(control_lib
    ${catkin_LIBRARIES}
    ${EXTRA_LIBS}
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
    casadi
    )

# TODO remove redundant libraries
add_executable(walking walking.cpp)
target_link_libraries(walking control_lib)

# Test
add_executable(test_casadi test_casadi.cpp)
target_link_libraries(test_casadi control_lib)