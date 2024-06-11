import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Set the path to different files and folders
    pkg_share = FindPackageShare(package='stanis_description').find('stanis_description')
    default_model_path = os.path.join(pkg_share, 'urdf/stanis_zed2i.urdf.xacro')

    # Launch configuration variables
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
        default_value='True',
        description='Use simulation time if true')
    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='use_joint_state_publisher',
        default_value='False',
        description='Use joint state publisher if true')

    # Publish static tf map -> odom
    #! Set the global EKF's initial state to this!
    # tf_map_odom = ExecuteProcess(
    #     cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
    #          '--frame-id', 'map',
    #          '--child-frame-id', 'stanis/odom',
    #          '--x', '0.0',
    #          '--y', '0.0',
    #          '--z', '0.85'],
    #     output='screen')

    # Publish static tf map -> orb2_map
    tf_map_orb2_map = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '--frame-id', 'map',
             '--child-frame-id', 'stanis/orb2_map',
             '--x', '0.0',
             '--y', '0.0',
             '--z', '0.85',
             '--qw', '0.999899',
             '--qx', '0.009532',
             '--qy', '-0.008918',
             '--qz', '0.005638'],
        output='screen')

    # Publish static tf odom -> zed2i_odom
    tf_odom_zed2i_odom = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '--frame-id', 'stanis/odom',
             '--child-frame-id', 'stanis/zed2i_odom',
             '--x', '0.0',
             '--y', '0.0',
             '--z', '0.0'],
        output='screen')

    # Publish static tf odom -> orb2_odom
    tf_odom_orb2_odom = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '--frame-id', 'stanis/odom',
             '--child-frame-id', 'stanis/orb2_odom',
             '--x', '0.0',
             '--y', '0.0',
             '--z', '0.0'],
        output='screen')

    # Publish the joint state values for the non-fixed joints in the URDF file
    start_joint_state_publisher_cmd = Node(
        condition=IfCondition(use_joint_state_publisher),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='stanis',
        parameters=[{'use_sim_time': use_sim_time}])

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='stanis',
        parameters=[{'use_sim_time': use_sim_time,
                     #'frame_prefix': 'stanis/',
                     'robot_description': Command(['xacro ', model])}],
        arguments=[default_model_path])

    # Create the launch description
    ld = LaunchDescription()

    # Add the launch options
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)

    # Add the executables
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    # ld.add_action(tf_map_odom)
    ld.add_action(tf_map_orb2_map)
    ld.add_action(tf_odom_zed2i_odom)
    ld.add_action(tf_odom_orb2_odom)

    return ld
