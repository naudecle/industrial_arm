import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the package share directory path
    pkg_share = get_package_share_directory('industrial_arm_description')
    
    # 2. Path to the main top-level master xacro file
    xacro_file = os.path.join(pkg_share, 'xacro', 'robot_industrial_arm.urdf.xacro')
    
    # 3. Process the macro expansions via the command line interface
    # This evaluates your properties, macros, and file inclusions dynamically
    robot_description_raw = Command(['xacro ', xacro_file])
    
    # 4. Robot State Publisher Node (Broadcasts /tf transformation frames)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )
    
    # 5. Joint State Publisher GUI Node (Creates the sliders to move the arm)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )
    
    # 6. RViz 2 Node (Graphical interface environment)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('industrial_arm_description'), 'config', 'robot_arm.rviz')],
    )
    
    # Return the execution pipeline structure
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])