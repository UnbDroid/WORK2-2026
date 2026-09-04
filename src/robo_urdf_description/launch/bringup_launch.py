"""
Launch file unificado do robô: substitui os terminais separados por um único
`ros2 launch <seu_pacote> bringup_launch.py`.

IMPORTANTE:
- O static_transform_publisher base_link -> lidar_link_1 foi removido de
  propósito. O joint fixo "Rigido" no seu xacro já publica essa transform
  (com a correção de rpy que você já aplicou). Rodar os dois ao mesmo tempo
  causa conflito de TF.
- Ajuste os caminhos de arquivo (serial_port, params_file, slam_params_file)
  conforme o seu ambiente.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 1. Descrição do robô (robot_state_publisher + joints do xacro)
    robo_urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robo_urdf_description'),
                'launch', 'robo.launch.py'
            )
        )
    )

    # 2. Driver do RPLiDAR S2
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch', 'sllidar_s2_launch.py'
            )
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': '1000000',
            'frame_id': 'lidar_link_1',
        }.items()
    )

    # 3. Filtro do scan (laser_filters)
    scan_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        parameters=[os.path.expanduser('~/filter_config/laser_filter.yaml')],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered'),
        ],
    )

    # 4. Odometria via laser (rf2o)
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[{
            'laser_frame_id': 'lidar_link_1',
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'use_sim_time': False,
        }],
        remappings=[
            ('scan', '/scan_filtered'),
            ('odom_rf2o', '/odom'),
        ],
    )

    # 5. SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': os.path.expanduser('~/slam_config/slam.yaml'),
            'use_sim_time': 'false',
        }.items()
    )

    # # 6. Nav2
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('nav2_bringup'),
    #             'launch', 'navigation_launch.py'
    #         )
    #     ),
    #     launch_arguments={
    #         'use_sim_time': 'false',
    #         'params_file': '/home/droid/nav2_config/nav2_params.yaml',
    #     }.items()
    # )

    

    # 7. Publish único do ground truth (atrasado alguns segundos pra dar
    #    tempo dos outros nós subirem e evitar perder a mensagem)
    ground_truth_pub = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once', '/base_pose_ground_truth',
                    'nav_msgs/msg/Odometry',
                    "{header: {frame_id: 'odom'}, child_frame_id: 'base_link'}"
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        robo_urdf_launch,
        sllidar_launch,
        scan_filter_node,
        rf2o_node,
        slam_launch,
 #       nav2_launch,
        ground_truth_pub
    ])
