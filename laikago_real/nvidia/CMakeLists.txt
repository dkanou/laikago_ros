find_package(Eigen3 3.3 REQUIRED NO_MODULE)
list(APPEND EXTRA_LIBS Eigen3::Eigen)

include_directories(include)

# Declare a C++ library
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
        casadi
        )

# TODO remove redundent libraries
add_executable(neutral_mode_low src/neutral_mode_low.cpp)
target_link_libraries(neutral_mode_low ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(neutral_mode_high src/neutral_mode_high.cpp)
target_link_libraries(neutral_mode_high ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(sensor_readings src/sensor_readings.cpp)
target_link_libraries(sensor_readings ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(walking src/walking.cpp)
target_link_libraries(walking ${EXTRA_LIBS} ${catkin_LIBRARIES} control_lib)

# Test
add_executable(test_casadi src/test_casadi.cpp)
target_link_libraries(test_casadi ${EXTRA_LIBS} ${catkin_LIBRARIES} control_lib)