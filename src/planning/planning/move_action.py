import rclpy
from plansys2_executor.ActionExecutorClient import ActionExecutorClient

class MoveAction(ActionExecutorClient):
    def __init__(self):
        super().__init__('move', 1.0)

    def do_work(self):
        args = self.get_arguments()
        robot = args[0]
        from_loc = args[1]
        to_loc = args[2]

        self.get_logger().info(f'[MoveAction] Movendo {robot} de {from_loc} para {to_loc}')

        # integrar com a navegação aqui (perguntar para analu e arthur)
        
        # quando a movimentação terminar com sucesso:
        self.finish(True, 1.0, f'Chegou em {to_loc}')

def main(args=None):
    rclpy.init(args=args)
    node = MoveAction()
    node.set_parameter(rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'move'))
    node.trigger_configure()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()