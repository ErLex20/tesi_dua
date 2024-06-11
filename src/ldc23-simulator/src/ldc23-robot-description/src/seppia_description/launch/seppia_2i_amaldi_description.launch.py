import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Set the path to different files and folders.
    pkg_share = FindPackageShare(package='seppia_description').find('seppia_description')
    default_model_path = os.path.join(pkg_share, 'urdf/seppia_zed2i.urdf.xacro')

    # Launch configuration variables specific to simulation
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')

    # Declare the launch arguments
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot URDF file')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation time if true')
    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='use_joint_state_publisher',
        default_value='False',
        description='Use joint state publisher if true')

    # Publish static tf map -> odom
    tf_map_seppia_map = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '--frame-id', 'map',
             '--child-frame-id', 'seppia/odom',
             '--x', '9.0',
             '--y', '4.5'],
        output='screen')

    # Publish static tf map -> orb2_map
    tf_map_orb2_map = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '--frame-id', 'map',
             '--child-frame-id', 'seppia/orb2_map',
             '--x', '12.0',
             '--y', '4.5',
             '--z', '0.12',
             '--qw', '0.999866',
             '--qx', '0.001752',
             '--qy', '-0.016252',
             '--qz', '-0.000405'],
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

    # Publish the joint state values for the non-fixed joints in the URDF file
    start_joint_state_publisher_cmd = Node(
        condition=IfCondition(use_joint_state_publisher),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='seppia',
        parameters=[{'use_sim_time': use_sim_time}])

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='seppia',
        parameters=[{'use_sim_time': use_sim_time,
                     #'frame_prefix': 'seppia/',
                     'robot_description': Command(['xacro ', model])}],
        arguments=[default_model_path])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)

    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(tf_map_seppia_map)
    ld.add_action(tf_map_orb2_map)
    ld.add_action(tf_odom_zed2i_odom)

    return ld
