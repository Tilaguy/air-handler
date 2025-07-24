from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Lanza Ignition Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', '/ruta/al/mundo.sdf'],
            output='screen'
        ),
        # Bridge para comandos de fuerza (ejemplo)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ros_ign_bridge'),
                '/launch/ign_bridge.launch.py'
            ]),
            launch_arguments={
                'ign_topic': '/world/default/model/imu_box/link/imu_body/wrench',
                'ros_topic': '/imu_box/apply_wrench',
                'bridge_type': 'geometry_msgs/msg/Wrench'
            }.items()
        )
    ])