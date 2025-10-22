import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Get the path to the rover_description package
    pkg_path = get_package_share_directory('rover_description')
    
    # Define the path to the URDF file
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'rover.urdf')
    
    # Process the URDF file
    robot_description_raw = xacro.process_file(urdf_file_path).toxml()
    
    # Define the path to the RViz config file (using the one from view_rover)
    rviz_config_path = os.path.join(pkg_path, 'launch', 'view_rover.rviz')

    # 1. Node for robot_state_publisher
    # Publishes TF transforms based on /joint_states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': False}]
    )

    # 2. Node for our forward kinematics
    # Subscribes to /cmd_vel and publishes /joint_states
    forward_kinematics_node = Node(
        package='rover_description',
        executable='forward_kinematics.py', # This is the entry point from setup.py
        name='forward_kinematics_node',
        output='screen'
    )

    # 3. Node for RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path] # Load our custom config file
    )

    # 4. Node for teleop_twist_keyboard
    # Publishes /cmd_vel
    teleop_twist_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e' # Launch in a new terminal window
    )

    return LaunchDescription([
        robot_state_publisher_node,
        forward_kinematics_node,
        rviz_node,
        teleop_twist_keyboard_node
    ])