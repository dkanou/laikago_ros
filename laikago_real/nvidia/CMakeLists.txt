find_package(Eigen3 3.3 REQUIRED NO_MODULE)
list(APPEND EXTRA_LIBS Eigen3::Eigen)

include_directories(include)

add_executable(neutral_mode_low src/neutral_mode_low.cpp)
target_link_libraries(neutral_mode_low ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(neutral_mode_high src/neutral_mode_high.cpp)
target_link_libraries(neutral_mode_high ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(sensor_readings src/sensor_readings.cpp)
target_link_libraries(sensor_readings ${EXTRA_LIBS} ${catkin_LIBRARIES})