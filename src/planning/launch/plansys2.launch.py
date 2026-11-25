import os
import re
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
        default_value='test-basicmove.pddl',
        description='arquivo de problema PDDL usado so pra inicializar o Problem Expert',
    )
    # Onde o robo comeca e termina: a arena tem duas areas de start e duas de finish
    start_arg = DeclareLaunchArgument(
        'start', default_value='1', choices=['1', '2'],
        description='area de inicio: 1 -> locstart1, 2 -> locstart2',
    )
    finish_arg = DeclareLaunchArgument(
        'finish', default_value='1', choices=['1', '2'],
        description='area de fim: 1 -> locfinish1, 2 -> locfinish2',
    )

    # Opcional: YAML do @Work Commander. Se for passado, e' convertido aqui mesmo
    # (planning/yaml_para_pddl.py) e o problem_file e' ignorado.
    task_yaml_arg = DeclareLaunchArgument(
        'task_yaml', default_value='',
        description='arquivo .yaml da prova (Commander); se vazio, usa problem_file',
    )

    locations_yaml = os.path.join(pkg_share, 'config', 'locations.yaml')

    def preparar_problema(context):
        start = LaunchConfiguration('start').perform(context)
        finish = LaunchConfiguration('finish').perform(context)
        task_yaml = LaunchConfiguration('task_yaml').perform(context)
        loc_start, loc_finish = f'locstart{start}', f'locfinish{finish}'
        logs = []

        if task_yaml:
            from planning.yaml_para_pddl import conferir_locations_yaml, traduzir
            with open(os.path.expanduser(task_yaml)) as f:
                doc = yaml.safe_load(f)
            texto, locations, avisos = traduzir(doc, loc_start, loc_finish)
            nome = f'problem-{doc["task_id"].lower()}'
            logs += [f'[planning] {task_yaml} convertido para PDDL'] + [f'[planning] AVISO: {a}' for a in avisos]
            faltando = conferir_locations_yaml(locations_yaml, locations)
            if faltando:
                logs.append(f'[planning] AVISO: faltam em locations.yaml: {", ".join(faltando)}')
        else:
            problem_file = LaunchConfiguration('problem_file').perform(context)
            with open(os.path.join(pkg_share, 'pddl', problem_file)) as f:
                texto = f.read()
            nome = os.path.splitext(problem_file)[0]
            # troca a area de inicio/fim do arquivo pela escolhida no launch
            for prefixo, novo_nome in (('locstart', loc_start), ('locfinish', loc_finish)):
                achados = set(re.findall(rf'\b{prefixo}\d\b', texto))
                if len(achados) == 1:
                    texto = re.sub(rf'\b{prefixo}\d\b', novo_nome, texto)
                elif len(achados) > 1:
                    logs.append(f'[planning] AVISO: {problem_file} usa {sorted(achados)}; '
                                f'{prefixo} nao foi trocado')
                else:
                    logs.append(f'[planning] AVISO: {problem_file} nao tem {prefixo}1/{prefixo}2; '
                                f'start/finish ignorados para ele')

        caminho = os.path.join(tempfile.gettempdir(), f'{nome}-s{start}-f{finish}.pddl')
        with open(caminho, 'w') as f:
            f.write(texto)
        logs.append(f'[planning] inicio: {loc_start} | fim: {loc_finish} | problema: {caminho}')

        # Incluir o launch base oficial do plansys2
        plansys2_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    plansys2_share, 'launch', 'plansys2_bringup_launch_distributed.py'
                )
            ),
            launch_arguments={
                'model_file': os.path.join(pkg_share, 'pddl', 'domain.pddl'),
                'problem_file': caminho,
                'autostart': 'true',
            }.items(),
        )
        return [LogInfo(msg=m) for m in logs] + [plansys2_cmd]


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
            'locations_file': locations_yaml,
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
            'locations_file': locations_yaml,
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
            'locations_file': locations_yaml,
        }],
    )

    place_in_container_action_node = LifecycleNode(
        package='planning',
        executable='place_in_container_action',
        name='place_in_container_action',
        namespace='',
        output='screen',
        parameters=[{
            'action_name': 'place-in-container',
            'locations_file': locations_yaml,
        }],
    )

    place_at_cavity_two_action_node = LifecycleNode(
            package='planning',
            executable='place_at_cavity_two_action',
            name='place_at_cavity_two_action',
            namespace='',
            output='screen',
            parameters=[{
                'action_name': 'place-at-cavity',
                'locations_file': locations_yaml,
            }],
        )

    # pick_from_container_action_node = LifecycleNode(
    #     package='planning',
    #     executable='pick_from_container_action',
    #     name='pick_from_container_action',
    #     namespace='',
    #     output='screen',
    #     parameters=[{
    #         'action_name': 'move-nao-cheio',
    #         'locations_file': locations_yaml,
    #     }],
    # )

    # place_from_at_shelf_action_node = LifecycleNode(
    #     package='planning',
    #     executable='place_from_at_shelf_action',
    #     name='place_from_at_shelf_action',
    #     namespace='',
    #     output='screen',
    #     parameters=[{
    #         'action_name': 'move-nao-cheio',
    #         'locations_file': locations_yaml,
    #     }],
    # )

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
            place_in_container_action_node,
            place_at_cavity_two_action_node
            # pick_from_container_action_node,
            # place_from_at_shelf_action_node,
            # controller_node,
        ],
    )

    return LaunchDescription([
        phase_arg,
        problem_file_arg,
        start_arg,
        finish_arg,
        task_yaml_arg,
        OpaqueFunction(function=preparar_problema),
        acoes,
    ])