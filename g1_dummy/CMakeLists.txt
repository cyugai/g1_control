cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(g1_dummy)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
#  cmake_modules
  tf2_ros
  rclcpp
  urdf
  kdl_parser
  robot_state_publisher
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_ipm
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_assets
  ocs2_msgs
  ocs2_ros_interfaces
  g1_interface
  hpp-fcl
)

find_package(ament_cmake REQUIRED)
#find_package(cmake_modules REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_ipm REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(g1_interface REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

###########
## Build ##
###########



link_directories(
  ${pinocchio_LIBRARY_DIRS}
)


# Main library
add_library(${PROJECT_NAME}
        src/gait/GaitKeyboardPublisher.cpp
        src/gait/GaitReceiver.cpp
        src/visualization/g1Visualizer.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# DDP-MPC node for legged robot
add_executable(g1_ddp_mpc  src/g1DdpMpcNode.cpp)
ament_target_dependencies(g1_ddp_mpc ${dependencies})
target_link_libraries(g1_ddp_mpc ${PROJECT_NAME})
target_compile_options(g1_ddp_mpc PRIVATE ${OCS2_CXX_FLAGS})

## SQP-MPC node for legged robot
add_executable(g1_sqp_mpc  src/g1SqpMpcNode.cpp)
ament_target_dependencies(g1_sqp_mpc ${dependencies})
target_link_libraries(g1_sqp_mpc ${PROJECT_NAME})
target_compile_options(g1_sqp_mpc PRIVATE ${OCS2_CXX_FLAGS})

## IPM-MPC node for legged robot
add_executable(g1_ipm_mpc  src/g1IpmMpcNode.cpp)
ament_target_dependencies(g1_ipm_mpc ${dependencies})
target_link_libraries(g1_ipm_mpc ${PROJECT_NAME})
target_compile_options(g1_ipm_mpc PRIVATE ${OCS2_CXX_FLAGS})

# Dummy node
add_executable(g1_dummy_node  src/g1DummyNode.cpp)
ament_target_dependencies(g1_dummy_node ${dependencies})
target_link_libraries(g1_dummy_node ${PROJECT_NAME})
target_compile_options(g1_dummy_node PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for target trajectories
add_executable(g1_target  src/g1PoseCommandNode.cpp)
ament_target_dependencies(g1_target ${dependencies})
target_link_libraries(g1_target ${PROJECT_NAME})
target_compile_options(g1_target PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for gait
add_executable(g1_gait_command  src/g1GaitCommandNode.cpp)
ament_target_dependencies(g1_gait_command ${dependencies})
target_link_libraries(g1_gait_command ${PROJECT_NAME})
target_compile_options(g1_gait_command PRIVATE ${OCS2_CXX_FLAGS})

find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target ocs2_legged_robot_ros")
    add_clang_tooling(
            TARGETS
            ${PROJECT_NAME}
            g1_ddp_mpc
            g1_sqp_mpc
            g1_ipm_mpc
            g1_dummy_node
            g1_target
            g1_gait_command
            SOURCE_DIRS src include
            CT_HEADER_DIRS include
            CF_WERROR
    )
endif(cmake_clang_tools_FOUND)


#############
## Install ##
#############

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        TARGETS
        g1_ddp_mpc
        g1_sqp_mpc
        g1_ipm_mpc
        g1_dummy_node
        g1_target
        g1_gait_command
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()