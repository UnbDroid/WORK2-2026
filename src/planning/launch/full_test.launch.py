import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    planning_dir = get_package_share_directory('planning') #pega o diretório de planning
    bringup_dir = get_package_share_directory('plansys2_bringup')

    problem_file = LaunchConfiguration('problem_file')

    declare_domain_cmd = DeclareLaunchArgument(
        'domain_file',
        default_value=os.path.join(planning_dir, 'pddl', 'domain.pddl'),
        description='Caminho absoluto para o domain.pddl')

    declare_problem_cmd = DeclareLaunchArgument(
        'problem_file',
        default_value=os.path.join(planning_dir, 'pddl', 'test-pickup-from-location.pddl'),
        description='Caminho absoluto para o problem.pddl a testar')

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'plansys2_bringup_launch_distributed.py')),
        launch_arguments={
            'model_file': os.path.join(planning_dir, 'pddl', 'domain.pddl'),
            'problem_file': PathJoinSubstitution([planning_dir, 'pddl', LaunchConfiguration('problem_file')])
        }.items())

    # declarar todos os nós de ação do robô
    actions = [
        #('move_action', 'move'),
        ('pick_from_location_action', 'pick-from-location'),
        #('pick_from_container_action', 'pick-from-container'),
        #('place_at_location_action', 'place-at-location'),
        #('place_in_container_action', 'place-in-container'),
        #('stack_action', 'stack'),
        #('unstack_action', 'unstack'),
    ]

    action_nodes = []
    for exec_name, action_name in actions:
        action_nodes.append(
            Node(
                package='planning',
                executable=exec_name,
                name=f'{exec_name}',
                namespace='',
                output='screen',
                parameters=[{'action_name': action_name}]
            )
        )

    # nó gerenciador que lê o problema e aciona a execução, faz o papl do terminal mas automaticamente pelo que vi
    controller_node = Node(
        package='planning',
        executable='planning_controller',
        name='planning_controller',
        output='screen',
        parameters=[{
            'problem_file': LaunchConfiguration('problem_file')
        }]
    )


    # abre o terminal interativo numa janela separada
    plansys2_terminal_cmd = Node(
        package='plansys2_terminal',
        executable='plansys2_terminal',
        name='plansys2_terminal',
        output='screen',
        prefix='xterm -hold -e'
    )

    ld = LaunchDescription() #objeto vazio que armazena ações do launch
    ld.add_action(declare_domain_cmd) # adiciona ação domain_file:=
    ld.add_action(declare_problem_cmd) # adiciona ação problem_file:=
    ld.add_action(bringup_cmd) #inicia infraestrutura plansys2

    for action in action_nodes: #adiciona nós de ação
        ld.add_action(action)

    ld.add_action(plansys2_terminal_cmd)

    return ld