import rclpy
from plansys2_executor.ActionExecutorClient import ActionExecutorClient  
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

# dicionário mapeando os pontos do pddl para as coordenadas no mapa
LOCATION_MAP = {
    'start': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0},
    'WS_1':  {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0},
    'WS_2':  {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0},
    'SH_1':  {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0},
    'PP':    {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0},
}

class MoveAction(ActionExecutorClient):
    def __init__(self):
        super().__init__('move', 1.0)
        self.navigator = BasicNavigator()

    def do_work(self):
        args = self.get_arguments()
        robot = args[0]
        from_loc = args[1]
        to_loc = args[2]

        if to_loc not in LOCATION_MAP:
            self.get_logger().error(f'[MoveAction] Localização {to_loc} não cadastrada!')
            self.finish(False, 0.0, f'Local desconhecido: {to_loc}')
            return

        # prepara a meta de navegação
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        coord = LOCATION_MAP[to_loc]
        target_pose.pose.position.x = float(coord['x'])
        target_pose.pose.position.y = float(coord['y'])
        target_pose.pose.orientation.z = float(coord['z'])
        target_pose.pose.orientation.w = float(coord['w'])

        # dispara a movimentação no Nav2
        if not self.navigator.isTaskComplete():
            self.get_logger().info(f'[MoveAction] Movendo {robot} para {to_loc}...')
            self.navigator.goToPose(target_pose)

        # checa o status do robô
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'[MoveAction] Chegou em {to_loc}!')
                self.finish(True, 1.0, f'Chegou em {to_loc}')
            else:
                self.get_logger().error(f'[MoveAction] Falha ao navegar para {to_loc}.')
                self.finish(False, 0.0, f'Falha ao ir para {to_loc}')
        else:
            feedback = self.navigator.getFeedback()
            if feedback:
                self.send_feedback(0.5, f'Restante: {feedback.distance_remaining:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = MoveAction()
    node.set_parameter(rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'move'))
    node.trigger_configure()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()