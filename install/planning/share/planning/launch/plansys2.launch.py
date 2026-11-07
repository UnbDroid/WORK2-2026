import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg_share = get_package_share_directory('planning')
    plansys2_share = get_package_share_directory('plansys2_bringup')

    # Qual fase da competicao o planning_controller deve montar e rodar
    phase_arg = DeclareLaunchArgument(
        'phase',
        default_value='bmt',
        description='fase a ser executada (bmt, btt1, btt2, att1, att2, amt, move_test)',
    )

    # Arquivo PDDL so para inicializar o Problem Expert oficial do plansys2
    problem_file_arg = DeclareLaunchArgument(
        'problem_file',
        default_value='actionmove-test-problem.pddl',
        description='arquivo de problema PDDL usado so pra inicializar o Problem Expert',
    )
    problem_file_path = PathJoinSubstitution(
        [pkg_share, 'pddl', LaunchConfiguration('problem_file')]
    )

    # Incluir o launch base oficial do plansys2
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                plansys2_share, 'launch', 'plansys2_bringup_launch_distributed.py'
            )
        ),
        launch_arguments={
            'model_file': os.path.join(pkg_share, 'pddl', 'domain.pddl'),
            'problem_file': problem_file_path,
            'autostart': 'true',
        }.items(),
    )

    locations_yaml = os.path.join(pkg_share, 'config', 'locations.yaml')

    # Node para a ação "move-cheio"
    move_cheio_action_node = LifecycleNode(
        package='planning',
        executable='move_action',
        name='action_move_cheio',
        namespace='',
        output='screen',
        parameters=[{
            'action_name': 'move-cheio',
            'locations_file': locations_yaml,
        }],
    )

    # Node para a ação "move-nao-cheio" (mesmo executavel, action_name diferente)
    move_nao_cheio_action_node = LifecycleNode(
        package='planning',
        executable='move_action',
        name='action_move_nao_cheio',
        namespace='',
        output='screen',
        parameters=[{
            'action_name': 'move-nao-cheio',
            'locations_file': locations_yaml,
        }],
    )

    pick_from_location_action_node = LifecycleNode(
        package='planning',
        executable='pick_from_location_action',
        name='pick_from_location_action',
        namespace='',
        output='screen',
        parameters=[{
            'action_name': 'pick-from-location',
        }],
    )

    place_at_location_action_node = LifecycleNode(
        package='planning',
        executable='place_at_location_action',
        name='place_at_location_action',
        namespace='',
        output='screen',
        parameters=[{
            'action_name': 'place-at-location',
        }],
    )

    stack_action_node = LifecycleNode(
        package='planning',
        executable='stack_action',
        name='stack_action',
        namespace='',
        output='screen',
        parameters=[{
            'action_name': 'stack',
        }],
    )

    # Nó gerenciador que popula o problema e aciona a execução do plano
    controller_node = Node(
        package='planning',
        executable='planning_controller',
        name='planning_controller',
        output='screen',
        parameters=[{
            'phase': LaunchConfiguration('phase')
        }],
    )

    # Os action nodes sobem 15s depois para não sobrecarregar a CPU no Raspberry Pi
    acoes = TimerAction(
        period=15.0,
        actions=[
            move_cheio_action_node,
            move_nao_cheio_action_node,
            pick_from_location_action_node,
            place_at_location_action_node,
            stack_action_node,
            # controller_node,
        ],
    )

    return LaunchDescription([
        phase_arg,
        problem_file_arg,
        plansys2_cmd,
        acoes,
    ])