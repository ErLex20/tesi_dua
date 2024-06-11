import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Set the path to different files and folders.
    pkg_share = FindPackageShare(package='axis_camera_description').find('axis_camera_description')
    default_model_path = os.path.join(pkg_share, 'urdf/axis_camera.urdf.xacro')

    # Launch configuration variables specific to simulation
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')

    # Declare the launch arguments
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='use_joint_state_publisher',
        default_value='True',
        description='Use joint state publisher if true')

    # Publish the joint state values for the non-fixed joints in the URDF file
    start_joint_state_publisher_cmd = Node(
        condition=IfCondition(use_joint_state_publisher),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='ptz',
        parameters=[{'use_sim_time': use_sim_time}])

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='ptz',
        parameters=[{'use_sim_time': use_sim_time,
                     'frame_prefix': 'ptz/',
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
    #ld.add_action(start_joint_state_publisher_cmd)

    return ld