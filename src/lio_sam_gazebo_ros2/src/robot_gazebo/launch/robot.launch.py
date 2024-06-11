"""
Stanis Gazebo simulation launch configuration.

Lorenzo Bianchi <lnz.bnc@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

October 14, 2023
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    ### Gazebo ###
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_gazebo'), 'launch'), '/gazebo.launch.py']),
    )

    ### robot ###
    robot_x = 0.0
    robot_y = 0.0
    robot_z = 0.1
    robot_R = 3.14
    sdf_robot_path = '/home/neo/.gazebo/models/robot/robot.sdf'
    spawn_robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'robot',
                                   '-file', sdf_robot_path,
                                   '-x', str(robot_x),
                                   '-y', str(robot_y),
                                   '-z', str(robot_z),
                                   '-R', str(robot_R)],
                        output='screen')

    ############################################################################################
    ############################################################################################
    ### Environment variables ###

    try:
        GMP = [os.environ['GAZEBO_MODEL_PATH'], ':/home/neo/workspace/.gazebo/models']
    except:
        GMP = [':/home/neo/workspace/.gazebo/models']
    env_GMP = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=GMP)

    # try:
    #     GPP = [os.environ['GAZEBO_PLUGIN_PATH'], ':/home/neo/workspace/build/entity_move_plugin']
    # except:
    #     GPP = ['/home/neo/workspace/build/entity_move_plugin']
    # env_GPP = SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=GPP)

    # try:
    #     LLP = [os.environ['LD_LIBRARY_PATH'], ':/home/neo/workspace/build/entity_move_plugin']
    # except:
    #     LLP = ['/home/neo/workspace/build/entity_move_plugin']
    # env_LLP = SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=LLP)

    ############################################################################################
    ############################################################################################

    return LaunchDescription([
        # env_GPP,
        env_GMP,
        # env_LLP,
        gazebo,
        spawn_robot,
    ])
