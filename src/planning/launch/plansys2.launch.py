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
    problem_file_arg = DeclareLaunchArgument(
        'problem_file',
        default_value='problem_att1.pddl',
        description='Nome do arquivo PDDL do problema a ser executado'
    )

    # incluir o launch base oficial do plansys2
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            plansys2_share, 'launch', 'plansys2_bringup_launch_distributed.py')
        ),
        launch_arguments={
            'model_file': os.path.join(pkg_share, 'pddl', 'domain.pddl')
        }.items()
    )

    # declarar todos os nós de ação do robô
    actions = [
        ('move_action', 'move'),
        #('pick_from_location_action', 'pick-from-location'),
        ('pick_from_container_action', 'pick-from-container'),
        ('place_at_location_action', 'place-at-location'),
        ('place_in_container_action', 'place-in-container'),
        ('stack_action', 'stack'),
        ('unstack_action', 'unstack'),
    ]

    action_nodes = []
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

    #nó que gerencia pegar o objeto de algum local, usando bt

    pick_from_location_action = Node(
    package='plansys2_bt_actions',
    executable='bt_action_node',
    name='pick_from_location',
    namespace='',
    output='screen',
    parameters=[
        os.path.join(pkg_share, 'config', 'params.yaml'),
        {
            'action_name': 'pick_from_location',
            'bt_xml_file': os.path.join(pkg_share, 'behaviour_trees', 'pick_from_location.xml')
        }
    ])

    # nó gerenciador que lê o problema e aciona a execução
    controller_node = Node(
        package='planning',
        executable='planning_controller',
        name='planning_controller',
        output='screen',
        parameters=[{
            'problem_file': LaunchConfiguration('problem_file')
        }]
    )

    return LaunchDescription([
        problem_file_arg,
        plansys2_cmd,
        *action_nodes,
        controller_node
    ])