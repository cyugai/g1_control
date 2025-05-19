import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config_file = get_package_share_directory('g1_dummy') + "/rviz/g1.rviz"
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='description_name',
            default_value='g1_description'
        ),
        launch.actions.DeclareLaunchArgument(
            name='multiplot',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='taskFile',
            default_value=get_package_share_directory(
                'g1_interface') + '/config/mpc/task.info'
        ),
        launch.actions.DeclareLaunchArgument(
            name='referenceFile',
            default_value=get_package_share_directory(
                'g1_interface') + '/config/command/reference_.info'
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=get_package_share_directory(
                'g1_legged_description') + '/urdf/g1_legged_control_.urdf'
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdfFileOrigin',
            default_value=get_package_share_directory(
                'g1_legged_description') + '/urdf/g1_legged_origin.urdf'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gaitCommandFile',
            default_value=get_package_share_directory(
                'g1_interface') + '/config/command/gait_.info'
        ),
        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=[launch.substitutions.LaunchConfiguration("urdfFile")],
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=["-d", rviz_config_file],
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('rviz'))
        ),
        launch_ros.actions.Node(
            package='g1_controllers',
            executable='joy_controller_node',
            name='normal_controller',
            output='screen',
            prefix= "",
            parameters=[
                {
                    'multiplot': launch.substitutions.LaunchConfiguration('multiplot')
                },
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'referenceFile': launch.substitutions.LaunchConfiguration('referenceFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'gaitCommandFile': launch.substitutions.LaunchConfiguration('gaitCommandFile')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='g1_dummy',
            executable='g1_gait_command',
            name='g1_gait_command',
            output='screen',
            parameters=[
                {
                    'multiplot': launch.substitutions.LaunchConfiguration('multiplot')
                },
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'referenceFile': launch.substitutions.LaunchConfiguration('referenceFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'gaitCommandFile': launch.substitutions.LaunchConfiguration('gaitCommandFile')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='g1_controllers',
            executable='g1_target_trajectories_publisher',
            name='g1_target',
            output='screen',
            parameters=[
                {
                    'referenceFile': launch.substitutions.LaunchConfiguration('referenceFile')
                },
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
            ]
        ),
        launch_ros.actions.Node(
            package='g1_mujoco_sim',
            executable='g1_sim',
            name='g1_sim',
            output='screen',
            parameters=[
                {
                    'multiplot': launch.substitutions.LaunchConfiguration('multiplot')
                },
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'referenceFile': launch.substitutions.LaunchConfiguration('referenceFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'gaitCommandFile': launch.substitutions.LaunchConfiguration('gaitCommandFile')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='g1_mujoco_sim',
            executable='joy',
            name='joy',
            output='screen',
            parameters=[
                {
                    'multiplot': launch.substitutions.LaunchConfiguration('multiplot')
                },
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'referenceFile': launch.substitutions.LaunchConfiguration('referenceFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'gaitCommandFile': launch.substitutions.LaunchConfiguration('gaitCommandFile')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
