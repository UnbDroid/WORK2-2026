import rclpy
from plansys2_executor.ActionExecutorClient import ActionExecutorClient  

class MoveAction(ActionExecutorClient):
    def __init__(self):
        super().__init__('pick_from_location', 1.0)

    def do_work(self):
        args = self.get_arguments()
        robot = args[0]
        obj = args[1]
        location = args[2]
        slot = args[3]

        if obj 

