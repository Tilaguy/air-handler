from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '/ros2_ws/src/sensor_sim/worlds/all_sensors.world',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        )
    ])
