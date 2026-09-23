from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value='/home/droid/nav2_config/nav2_params.yaml',
        description='Full path to the Nav2 parameters file'
    )

    # Remappings padrão usados pelos nós do Nav2
    tf_remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    # --------------------------------------------------
    # Controller Server + Local Costmap
    # Publica comandos de navegação em /cmd_vel_nav
    # --------------------------------------------------
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=tf_remappings + [
            ('cmd_vel', 'cmd_vel_nav'),
        ],
    )

    # --------------------------------------------------
    # Path Smoother
    # --------------------------------------------------
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=tf_remappings,
    )

    # --------------------------------------------------
    # Planner Server + Global Costmap
    # --------------------------------------------------
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=tf_remappings,
    )

    # --------------------------------------------------
    # Nav2 Behaviors / Recoveries
    # Também publica comandos em /cmd_vel_nav
    # --------------------------------------------------
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=tf_remappings + [
            ('cmd_vel', 'cmd_vel_nav'),
        ],
    )

    # --------------------------------------------------
    # Behavior Tree Navigator
    # --------------------------------------------------
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=tf_remappings,
    )

    # --------------------------------------------------
    # Velocity Smoother
    #
    # Entrada:
    #   /cmd_vel_nav
    #
    # A saída continua definida pelo YAML
    # (no seu caso /cmd_vel_smoothed)
    # --------------------------------------------------
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=tf_remappings + [
            ('cmd_vel', 'cmd_vel_nav'),
        ],
    )

    # --------------------------------------------------
    # Collision Monitor
    #
    # Não fazemos cmd_vel -> cmd_vel_nav aqui.
    # Os tópicos de entrada/saída são definidos pelo YAML.
    #
    # Esperado:
    # /cmd_vel_smoothed -> collision_monitor -> /cmd_vel
    # --------------------------------------------------
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=tf_remappings,
    )

#Waypoint_follower

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=tf_remappings,
    )

    # --------------------------------------------------
    # Lifecycle Manager
    # --------------------------------------------------
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,

        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        velocity_smoother,
        collision_monitor,
        bt_navigator,
        waypoint_follower,

        lifecycle_manager,
    ])
