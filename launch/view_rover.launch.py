import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():

    # Get the path to the rover_description package
    pkg_path = get_package_share_directory('rover_description')
    
    # Define the path to the URDF file
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'rover.urdf')
    
    # Process the URDF file
    robot_description_raw = xacro.process_file(urdf_file_path).toxml()
    
    # Define the path to the RViz config file
    rviz_config_path = os.path.join(pkg_path, 'launch', 'view_rover.rviz')

    # 1. Node for robot_state_publisher
    # This node publishes the robot's state (TF transforms) based on the URDF
    # and the joint states published by joint_state_publisher.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': False}] # Set use_sim_time to False for live control
    )

    # 2. Node for joint_state_publisher_gui
    # This node provides a GUI with sliders to manually control
    # the non-fixed joints of the robot.
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # 3. Node for RViz
    # This node launches the RViz visualization tool.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path] # Load our custom config file
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])