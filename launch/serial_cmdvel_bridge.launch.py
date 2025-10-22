from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyACM0',
        description='Serial port to connect Mega2560'
    )
    baud_arg = DeclareLaunchArgument(
        'baud', default_value='115200',
        description='Baud rate'
    )

    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')

    return LaunchDescription([
        port_arg,
        baud_arg,

        # ⛔ teleop_twist_keyboard는 주석처리 — 별도 터미널에서 실행
        # Node(
        #     package='teleop_twist_keyboard',
        #     executable='teleop_twist_keyboard',
        #     name='teleop',
        #     output='screen',
        # ),

        # ✅ Mega 시리얼 브리지 노드
        Node(
            package='ros_mega',
            executable='serial_cmdvel_bridge',
            name='serial_cmdvel_bridge',
            output='screen',
            parameters=[{
                'port': port,
                'baud': baud,
                'lin_thresh': 0.05,
                'ang_thresh': 0.10,
                'deadman_sec': 0.8,
            }]
        )
    ])
