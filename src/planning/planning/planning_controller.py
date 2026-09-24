#!/usr/bin/env python3
'''
Lê o parâmetro 'phase' (bmt, btt1, btt2, att1, att2, amt) e:
  1. Limpa o conhecimento do Problem Expert.
  2. Popula instâncias e predicados iniciais da fase escolhida.
  3. Define o goal da fase.
  4. Pede um plano ao Planner.
  5. Executa o plano com o Executor.

Uma função de setup por fase
retorno: (lista_de_instancias, lista_de_predicados, goal)
    instancia ex: "robot1 robot"
    predicado ex: "(at-robot robot1 start)"
    goal ex: "(and(at-robot robot1 ws1))"
'''

from enum import Enum, auto

import rclpy
from rclpy.node import Node

from plansys2_support_py.ProblemExpertClient import ProblemExpertClient
from plansys2_support_py.PlannerClient import PlannerClient
from plansys2_support_py.ExecutorClient import ExecutorClient
from plansys2_support_py.Parser import Parser


class State(Enum):
    SETUP = auto()
    PLAN = auto()
    EXECUTE = auto()
    DONE = auto()
    ERROR = auto()


def setup_move_test(_problem_expert_):
    instances = [
        'robot1 robot',
        'start location',
        'wp1 location'
    ]
    predicates = [
        '(at-robot robot1 start)'
    ]
    goal = '(and (at-robot robot1 wp1))'
    return instances, predicates, goal


def setup_bmt(_problem_expert):
    instances = [
        'robot1 robot',
        
        'start location',
        'ws1 location',
        'ws4 location',
        'ws8 location',

        's1 slot',
        's2 slot',
        's3 slot',

        'obj1 object',
        'obj2 object',
        'obj4 object',
        'obj7 object',
    ]
    predicates = [
        '(at-robot robot1 start)',
        
        '(slot-free robot1 s1)',
        '(slot-free robot1 s2)',
        '(slot-free robot1 s3)',

        '(obj-at obj1 ws1)',
        '(obj-at obj2 ws1)',
        '(obj-at obj4 ws1)',
        '(obj-at obj ws1)',
    ]
    goal = '(and (obj-at obj1 ws4)(obj-at obj4 ws4)(obj-at obj2 ws8)(obj-at obj7 ws1))'
    return instances, predicates, goal


def setup_btt1(_problem_expert):
    instances = [
        'robot1 robot',
        
        'start location',
        'ws3 location',
        'ws4 location',
        'ws5 location',

        's1 slot',
        's2 slot',
        's3 slot',

        'obj1 object',
        'obj3 object',
        'obj5 object',
    ]
    predicates = [
        '(at-robot robot1 start)',
        
        '(slot-free robot1 s1)',
        '(slot-free robot1 s2)',
        '(slot-free robot1 s3)',

        '(obj-at obj1 ws4)',
        '(obj-at obj3 ws5)',
        '(obj-at obj5 ws3)',
    ]
    goal = '(and (obj-at obj1 ws4)(obj-at obj3 ws5)(obj-at obj5 ws3))'
    return instances, predicates, goal


def setup_btt2(_problem_expert):
    instances, predicates, goal = [], [], '(and )'
    return instances, predicates, goal


def setup_att1(_problem_expert):
    instances, predicates, goal = [], [], '(and )'
    return instances, predicates, goal


def setup_att2(_problem_expert):
    instances, predicates, goal = [], [], '(and )'
    return instances, predicates, goal


def setup_amt(_problem_expert):
    instances = [
    ]
    predicates = [
    ]
    goal = ''
    return instances, predicates, goal


PHASE_SETUP = {
    'move_test': setup_move_test,
    'bmt': setup_bmt,
    'btt1': setup_btt1,
    'btt2': setup_btt2,
    'att1': setup_att1,
    'att2': setup_att2,
    'amt': setup_amt,
}


class PlanningController(Node):

    def __init__(self):
        super().__init__('planning_controller')

        self.declare_parameter('phase', 'bmt')
        self.phase = self.get_parameter('phase').get_parameter_value().string_value

        if self.phase not in PHASE_SETUP:
            self.get_logger().error(
                f"Fase '{self.phase}' desconhecida. "
                f"Opções válidas: {list(PHASE_SETUP.keys())}")
            raise SystemExit(1)

        self.problem_expert = ProblemExpertClient()
        self.planner_client = PlannerClient()
        self.executor_client = ExecutorClient()

        self.plan = None
        self.state = State.SETUP

        # A FSM avança sozinha via timer, mesma ideia do controller oficial
        self.timer = self.create_timer(0.5, self.step)

    def step(self):
        if self.state == State.SETUP:
            self._do_setup()
        elif self.state == State.PLAN:
            self._do_plan()
        elif self.state == State.EXECUTE:
            self._do_execute()
        elif self.state in (State.DONE, State.ERROR):
            self.timer.cancel()

    def _do_setup(self):
        self.get_logger().info(f'Inicializando conhecimento para a fase: {self.phase}')

        self.problem_expert.clear_problem_knowledge()

        instances, predicates, goal = PHASE_SETUP[self.phase](self.problem_expert)

        for inst in instances:
            self.problem_expert.add_problem_instance(Parser.param_from_string(inst))

        for pred in predicates:
            self.problem_expert.add_problem_predicate(
                Parser.node_from_string_predicate(pred))

        self.problem_expert.add_problem_goal(Parser.tree_from_string(goal))

        self.state = State.PLAN

    def _do_plan(self):
        self.get_logger().info('Calculando plano...')
        # Confira a assinatura exata: pode ser get_plan() sem argumentos
        # (pega domínio/problema atuais do Problem/Domain Expert) ou pedir
        # domain/problem como string, dependendo da versão instalada.
        self.plan = self.planner_client.get_plan()

        if self.plan is None:
            self.get_logger().error(
                f'Não foi possível gerar um plano para a fase {self.phase}. '
                'Confira se o goal é alcançável a partir do estado inicial '
                '(o mesmo tipo de checagem que você já fez via BFS em Python).')
            self.state = State.ERROR
            return

        n_actions = len(self.plan.items) if hasattr(self.plan, 'items') else '?'
        self.get_logger().info(f'Plano encontrado com {n_actions} ações.')
        self.state = State.EXECUTE

    def _do_execute(self):
        self.get_logger().info('Executando plano...')
        success = self.executor_client.execute_plan(self.plan)

        if success:
            self.get_logger().info(f'Fase {self.phase} concluída com sucesso!')
        else:
            self.get_logger().error(f'Falha na execução da fase {self.phase}.')

        self.state = State.DONE


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PlanningController()
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
