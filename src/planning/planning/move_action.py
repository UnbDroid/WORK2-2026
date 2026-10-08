'''
espera que o último argumento da ação PDDL seja o destino, por ex: (move r1 start sh1)

as coordenadas de cada 'location' vêm de locations.yaml, carregado como parâmetros ROS no launch
'''

import math
import yaml
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from plansys2_support_py.ActionExecutorClient import ActionExecutorClient


NAV2_STATUS_SUCCEEDED = 4  # action_msgs/msg/GoalStatus.STATUS_SUCCEEDED


class MoveAction(ActionExecutorClient):

    def __init__(self):
        print('>>> MoveAction.__init__ COMEÇOU', flush=True)
        super().__init__('move', 0.5)
        print('>>> super().__init__ TERMINOU', flush=True)

        self._locations = {}
        self._load_locations()
        print('>>> _load_locations TERMINOU', flush=True)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        print('>>> ActionClient criado', flush=True)
        self._nav_done = False
        self._nav_success = False

    def _load_locations(self):
        self.declare_parameter('locations_file', '')

        locations_file = self.get_parameter(
            'locations_file'
        ).get_parameter_value().string_value

        if not locations_file:
            self.get_logger().error(
                "Parâmetro 'locations_file' não informado."
            )
            return

        self.get_logger().info(
            f'Carregando localizações de: {locations_file}'
        )

        try:
            with open(locations_file, 'r') as f:
                data = yaml.safe_load(f)

            locations = data['/**']['ros__parameters']['locations']

            for name, value in locations.items():
                if isinstance(value, (list, tuple)) and len(value) == 3:
                    self._locations[name] = tuple(
                        float(v) for v in value
                    )

            self.get_logger().info(
                f'Localizações carregadas: {list(self._locations.keys())}'
            )

        except Exception as e:
            self.get_logger().error(
                f'Erro ao carregar locations.yaml: {e}'
            )

    def on_activate(self, state):
        self._nav_done = False
        self._nav_success = False

        args = self.get_arguments()
        destination = args[-1]  # ajustar o índice se necessário

        if destination not in self._locations:
            self.get_logger().error(
                f"Local '{destination}' não encontrado em locations.yaml. "
                f"Locais conhecidos: {list(self._locations.keys())}")
            self._nav_done = True
            self._nav_success = False
            return super().on_activate(state)

        x, y, theta = self._locations[destination]
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f'[move] navegando até {destination} ({x}, {y})')

        self._nav_client.wait_for_server()
        send_goal_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_cb)
        send_goal_future.add_done_callback(self._nav_goal_response_cb)

        return super().on_activate(state)

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejeitou o goal de navegação')
            self._nav_done = True
            self._nav_success = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.send_feedback(0.5, f'Nav2: {remaining:.2f}m restantes')

    def _nav_result_cb(self, future):
        result = future.result()
        self._nav_done = True
        self._nav_success = (result.status == NAV2_STATUS_SUCCEEDED)

    def do_work(self):
        if not self._nav_done:
            return
        if self._nav_success:
            self.finish(True, 1.0, 'Move concluído (Nav2)')
        else:
            self.finish(False, 0.0, 'Move falhou (Nav2)')


def main(args=None):
    rclpy.init(args=args)

    node = MoveAction()

    result = node.trigger_configure()

    node.get_logger().info(
        f'Resultado do configure: {result}'
    )

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()