import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg_share = get_package_share_directory('planning')
    plansys2_share = get_package_share_directory('plansys2_bringup')

    # qual fase da competicao o planning_controller deve montar e rodar
    phase_arg = DeclareLaunchArgument(
        'phase',
        default_value='bmt',
        description='fase a ser executada (bmt, btt1, btt2, att1, att2, amt, move_test)'
    )

    # arquivo PDDL so para inicializar o Problem Expert oficial do plansys2
    # o conteudo real do problema e definido em runtime pelo planning_controller
    problem_file_arg = DeclareLaunchArgument(
        'problem_file',
        default_value='actionmove-test-problem.pddl',
        description='arquivo de problema PDDL usado so pra inicializar o Problem Expert'
    )
    problem_file_path = PathJoinSubstitution(
        [pkg_share, 'pddl', LaunchConfiguration('problem_file')]
    )

    # incluir o launch base oficial do plansys2
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            plansys2_share, 'launch', 'plansys2_bringup_launch_distributed.py')
        ),
        launch_arguments={
            'model_file': os.path.join(pkg_share, 'pddl', 'domain.pddl'),
            'problem_file': problem_file_path,
            'autostart': 'true',
        }.items()
    )

    locations_yaml = os.path.join(
        pkg_share,
        'config',
        'locations.yaml'
    )

    move_action_node = LifecycleNode(
        package='planning',
        executable='move_action',
        name='action_move_action',
        namespace='',
        output='screen',
        parameters=[{
            'action_name': 'move',
            'locations_file': os.path.join(
                pkg_share,
                'config',
                'locations.yaml'
            ),
    }]
)


    # no gerenciador que popula o problema e aciona a execucao do plano
    controller_node = Node(
        package='planning',
        executable='planning_controller',
        name='planning_controller',
        output='screen',
        parameters=[{
            'phase': LaunchConfiguration('phase')
        }]
    )

    return LaunchDescription([
    phase_arg,
    problem_file_arg,
    plansys2_cmd,
    move_action_node,
    # controller_node,
])
