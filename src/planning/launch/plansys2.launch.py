import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
        }.items()
    )

    move_action_node = LifecycleNode(
        package='planning',
        executable='move_action',
        name='action_move_action',
        namespace='',
        output='screen',
        parameters=[{'action_name': 'move'},
                    os.path.join(pkg_share, 'config', 'locations.yaml')]
    )

    '''# demais acoes do robo
    actions = [
        ('pick_from_container_action', 'pick-from-container'),
        ('place_at_location_action', 'place-at-location'),
        ('place_in_container_action', 'place-in-container'),
        ('stack_action', 'stack'),
        ('unstack_action', 'unstack'),
    ]'''

    action_nodes = [move_action_node]
    for exec_name, action_name in actions:
        action_nodes.append(
            LifecycleNode(
                package='planning',
                executable=exec_name,
                name=f'action_{exec_name}',
                namespace='',
                output='screen',
                parameters=[{'action_name': action_name}]
            )
        )

    # no que gerencia pegar o objeto de algum local, usando bt
    pick_from_location_action = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='pick_from_location',
        namespace='',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'params.yaml'),
            {
                'action_name': 'pick-from-location',
                'bt_xml_file': os.path.join(pkg_share, 'behaviour_trees', 'pick_from_location.xml'),
            }
        ]
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
        *action_nodes,
        #controller_node
    ])