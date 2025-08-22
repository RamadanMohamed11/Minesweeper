from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Define the path to the URDF file
    urdf_file = os.path.join(
        os.getenv('COLCON_PREFIX_PATH', ''), 'src', 'my_robot_package', 'urdf', 'my_robot.urdf'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        )
    ])

