cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(g1_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        rclcpp
        std_msgs
        g1_interface
        g1_wbc
        g1_estimation
        g1_dummy
        ocs2_ros_interfaces
        ocs2_self_collision
        ocs2_self_collision_visualization
        geometry_msgs
        ocs2_msgs
        angles
        hpp-fcl
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(g1_interface REQUIRED)
find_package(g1_wbc REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(g1_estimation REQUIRED)
find_package(g1_dummy REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)
find_package(angles REQUIRED)
find_package(hpp-fcl REQUIRED)

###########
## Build ##
###########

# Main library
add_library(${PROJECT_NAME}
        src/g1Controller.cpp
)
target_link_libraries(${PROJECT_NAME}
        
        ${pinocchio_LIBRARIES}
)

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# Target trajectories publish node
add_executable(g1_target_trajectories_publisher
        src/TargetTrajectoriesPublisher.cpp
        )
add_dependencies(g1_target_trajectories_publisher
        ${PROJECT_NAME}
        )
target_link_libraries(g1_target_trajectories_publisher
        ${PROJECT_NAME}
        )
target_compile_options(g1_target_trajectories_publisher PRIVATE ${OCS2_CXX_FLAGS})


# Cheat Controller Node
add_executable(cheat_controller_node
        src/CheatControllerNode.cpp
        )
add_dependencies(cheat_controller_node
        ${PROJECT_NAME}
        )
target_link_libraries(cheat_controller_node
        ${PROJECT_NAME}
        )
target_compile_options(cheat_controller_node PRIVATE ${OCS2_CXX_FLAGS})


# Normal Controller Node
add_executable(normal_controller_node
        src/NormalControllerNode.cpp
        )
add_dependencies(normal_controller_node
        ${PROJECT_NAME}
        )
target_link_libraries(normal_controller_node
        ${PROJECT_NAME}
        )
target_compile_options(normal_controller_node PRIVATE ${OCS2_CXX_FLAGS})

# Joy Controller Node
add_executable(joy_controller_node
        src/JoyControllerNode.cpp
        )
add_dependencies(joy_controller_node
        ${PROJECT_NAME}
        )
target_link_libraries(joy_controller_node
        ${PROJECT_NAME}
        )
target_compile_options(joy_controller_node PRIVATE ${OCS2_CXX_FLAGS})

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
            g1_target_trajectories_publisher
            cheat_controller_node
            normal_controller_node
            joy_controller_node
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
        g1_target_trajectories_publisher
        cheat_controller_node
        normal_controller_node
        joy_controller_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
