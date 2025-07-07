from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '/ros2_ws/src/test_world/worlds/imu_test.world',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        )
    ])

