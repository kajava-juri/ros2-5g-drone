from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

NAMESPACE = 'mavros'
FCU_URL = 'udp://:14540@localhost:14557'  # MAVROS listens on 14540, sends to PX4 on 14557
GCS_URL = ''
TGT_SYSTEM = 1
TGT_COMPONENT = 1
FCU_PROTOCOL = 'v2.0'
CONFIG_DIR = get_package_share_directory('5g_drone')
config = os.path.join(CONFIG_DIR, 'config', 'fc_config.yaml')


def generate_launch_description():
    return LaunchDescription([
        # MAVROS
        Node(
            package='mavros',
            executable='mavros_node',
            namespace=NAMESPACE,
            parameters=[
                config,
                {'fcu_url': FCU_URL},
                {'gcs_url': GCS_URL},
                {'tgt_system': TGT_SYSTEM},
                {'tgt_component': TGT_COMPONENT},
                {'fcu_protocol': FCU_PROTOCOL}
            ],
            output='screen'
        ),

        # Joystick
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),

        # Joystick to velocity
        Node(
            package='5g_drone',
            executable='joystick_offb',
            name='joystick_control',
            remappings=[
                ('/joystick/cmd_vel', '/desired_velocity')
            ]
        ),

        # # Obstacle avoidance
        # Node(
        #     package='your_package',
        #     executable='obstacle_avoidance',
        #     name='obstacle_avoidance'
        # ),

        # # LiDAR driver (example: RPLidar)
        # Node(
        #     package='rplidar_ros',
        #     executable='rplidar_node',
        #     name='rplidar',
        #     parameters=[{
        #         'serial_port': '/dev/ttyUSB0',
        #         'frame_id': 'laser_frame'
        #     }]
        # )

    ])
