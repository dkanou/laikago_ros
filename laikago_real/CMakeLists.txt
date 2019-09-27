cmake_minimum_required(VERSION 2.8.3)
project(laikago_real)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    roscpp
    tf
    geometry_msgs
    laikago_msgs
)

catkin_package(
    CATKIN_DEPENDS 
    laikago_msgs
)

include_directories(
    include
    $ENV{Laikago_SDK}/include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

add_compile_options(-std=c++14)

link_directories($ENV{Laikago_SDK}/lib)
set(EXTRA_LIBS liblaikago_comm.so lcm)
set(CMAKE_CXX_FLAGS "-no-pie")

add_executable(position_lcm example/position_mode.cpp)
target_link_libraries(position_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(velocity_lcm example/velocity_mode.cpp)
target_link_libraries(velocity_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(torque_lcm example/torque_mode.cpp)
target_link_libraries(torque_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(walk_lcm example/walk_mode.cpp)
target_link_libraries(walk_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})

# nvidia
add_subdirectory(nvidia)
