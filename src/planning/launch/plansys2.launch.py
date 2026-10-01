import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg_share = get_package_share_directory('planning')
    plansys2_share = get_package_share_directory('plansys2_bringup')

    # selecionar qual das 6 tarefas executar
    phase_arg = DeclareLaunchArgument(
        'phase',
        default_value='bmt',
        description='fase a ser executada (bmt, btt1, btt1, amt, att1, att2, move_test)'
    )

    # arquivo PPDL só para inicializar o Problem Expert oficial do plansys2
    # o arquivo de problema real será lido pelo nó planning_ontroller
    problem_file_arg = DeclareLaunchArgument(
        'problem_file',
        default_value='actionmove-test-problem.yaml',
        description='arquivo de problema PDDL usado só pra inicializar o Problem Expert'
    )
    problem_file_path = PathJoinSubstitution(
        [pkg_share, 'pddl', LaunchConfiguration('problem_file')
    ])

    # incluir o launch base oficial do plansys2
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            plansys2_share, 'launch', 'plansys2_bringup_launch_distributed.py')
        ),
        launch_arguments={
            'model_file': os.path.join(pkg_share, 'pddl', 'domain.pddl')
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

    # demais ações do robô
    actions = [
        ('pick_from_location_action', 'pick-from-location'),
        ('pick_from_container_action', 'pick-from-container'),
        ('place_at_location_action', 'place-at-location'),
        ('place_in_container_action', 'place-in-container'),
        ('stack_action', 'stack'),
        ('unstack_action', 'unstack'),
    ]

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

    # nó que gerenia pegar o objeto de algu loal, usando bt
    pick_from__loation_action = Node(
        package='plansys2_bt_actions',
        executable='pick_from_location',
        name='',
        namespace='',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'params.yaml'),
            {
                'action_name': 'pick-from-location',
                'bt_xml_file': os.path.join(pkg_share, 'behavior_trees', 'pick_from_location.xml'),
            }
        ]
    )


    # nó gerenciador que lê o problema e aciona a execução do plano
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
        plansys2_cmd,
        *action_nodes,
        move_action_node,
        controller_node
    ])