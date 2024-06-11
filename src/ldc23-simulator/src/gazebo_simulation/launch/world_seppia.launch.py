"""
Seppia Gazebo simulation launch configuration.

Lorenzo Bianchi <lnz.bnc@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

October 14, 2023
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from math import pi

import xacro


def generate_launch_description():
    ### Gazebo ###
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_simulation'), 'launch'), '/gazebo.launch.py']),
    # )

    ############################################################################################
    ############################################################################################
    ### Competition field ###

    # field_x = 0.0
    # field_y = 0.0
    # field_Y = 0.0
    # # sdf_field_path = '/home/neo/workspace/src/ldc23-simulator/config/gazebo/models/competition_field/model.sdf'
    # sdf_field_path = '/home/neo/.gazebo/models/dynamic_world/world.model'
    # spawn_field = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                    arguments=['-entity', 'competition_field',
    #                               '-file', sdf_field_path,
    #                               '-x', str(field_x),
    #                               '-y', str(field_y),
    #                               '-Y', str(field_Y)],
    #                    output='screen')

    ############################################################################################
    ############################################################################################
    ### Seppia ###

    seppia_x = 0.0
    seppia_y = 0.0
    seppia_z = 0.1
    seppia_Y = 0.0
    # sdf_seppia_path = '/home/neo/workspace/src/ldc23-simulator/config/gazebo/models/seppia/model.sdf'
    sdf_seppia_path = '/home/neo/.gazebo/models/seppia/model.sdf'
    spawn_seppia = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'seppia',
                                   '-file', sdf_seppia_path,
                                   '-x', str(seppia_x),
                                   '-y', str(seppia_y),
                                   '-z', str(seppia_z),
                                   '-Y', str(seppia_Y)],
                        output='screen')
    seppia_path = os.path.join(get_package_share_directory('seppia_description'))
    xacro_file = os.path.join(seppia_path, 'urdf', 'seppia_zed2i.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': True} #, 'frame_prefix': 'seppia/'}
    robot_state_publisher_seppia = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    ############################################################################################
    ############################################################################################
    ### Static TFs ###

    # Publish static tf map -> seppia/map
    tf_map_seppia_map = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '--frame-id', 'map',
             '--child-frame-id', 'seppia/map',
             '--x', '2.21',
             '--y', '7.83'],
        output='screen')

    # Publish static tf map -> odom
    tf_seppia_map_odom = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '--frame-id', 'seppia/map',
             '--child-frame-id', 'seppia/odom'],
        output='screen')

    # Publish static tf odom -> zed2i_odom
    tf_odom_zed2i_odom = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '--frame-id', 'seppia/odom',
             '--child-frame-id', 'seppia/zed2i_odom',
             '--x', '0.159',
             '--y', '0.060',
             '--z', '0.130',
             '--qw', '0.984',
             '--qx', '0.000',
             '--qy', '-0.177',
             '--qz', '0.000'],
        output='screen')

    ############################################################################################
    ############################################################################################
    ### Environment variables ###

    try:
        GPP = [os.environ['GAZEBO_PLUGIN_PATH'],
               ':/home/neo/workspace/build/gazebo_plugins']
    except:
        GPP = ['/home/neo/workspace/build/gazebo_plugins']
    env_GPP = SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=GPP)

    try:
        GMP = [os.environ['GAZEBO_MODEL_PATH'], ':/home/neo/.gazebo/models']
    except:
        GMP = ['/home/neo/.gazebo/models']
    env_GMP = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=GMP)

    try:
        LLP = [os.environ['LD_LIBRARY_PATH'],
               ':/home/neo/workspace/build/gazebo_plugins']
    except:
        LLP = ['/home/neo/workspace/build/gazebo_plugins']
    env_LLP = SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=LLP)

    ############################################################################################
    ############################################################################################

    return LaunchDescription([
        env_GPP,
        env_GMP,
        env_LLP,

        robot_state_publisher_seppia,

        spawn_seppia,

        tf_map_seppia_map,
        tf_seppia_map_odom,
        tf_odom_zed2i_odom
    ])
