from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    sensor_name = 'force'
    world_file = f'/ros2_ws/src/sensor_sim/worlds/{sensor_name}_test.world'

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                world_file,
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        )
    ])
