import rclpy
from .gripper_primitives import GripperPrimitives
from enum import Enum, auto
import time

class PickCubeStep(Enum):
    START = auto()
    RECUAR_INICIAL = auto()
    GIRAR_AREA = auto()
    VERTICAL_AREA = auto() #podem ser simultâneos girar e vertical
    TRANSLADAR = auto()
    AVANCAR = auto()
    FECHAR = auto()
    RECUAR_CUBO = auto()
    GIRAR_CHASSI = auto() #podem ser simultaneos girar e vertical SE ALTURA FOR MAIOR QUE DO ROBO, a
    VERTICAL_CHASSI = auto()
    ABRIR = auto()
    VERTICAL_INICIAL = auto()
    FINISH = auto()


class PickCubeFromLocationAction(GripperPrimitives):
    def __init__(self):
        super().__init__('pick-from-location') #nome precisa bater com nome da action no pddl domain
        self.step = PickCubeStep.START

    def do_work(self):
        args = self.get_arguments()  # [robot, cube, height, slot] guarda os parâmetros mandados pela ação

        if self.step == PickCubeStep.START:
            self.step = PickCubeStep.RECUAR_INICIAL
            self.send_feedback(0.0, 'recuando')
            #incluir aqui funcao que faz o robo recuar, ver questão de feedback para concluir, já que isso é realizado por send_commands(), que publica no tópico da garra que não tange as rodas e sua movimentação
            time.sleep(1)
            self._pending_reply = 'move' #temp
            self._current_reply = 'move' 
            self._reply_received = True 

        elif self.step == PickCubeStep.RECUAR_INICIAL and self.command_finished():
            self.step = PickCubeStep.GIRAR_AREA
            self.send_feedback(0.18, 'girando para area de cubo')
            self.send_command('frente') #mudar comando para frente

        elif self.step == PickCubeStep.GIRAR_AREA and self.command_finished():
            self.step = PickCubeStep.VERTICAL_AREA
            self.send_feedback(0.27, 'movendo para altura da mesa')
            self.send_command(self.height_to_cmd(args[3]))

        elif self.step == PickCubeStep.VERTICAL_AREA and self.command_finished():
            self.step = PickCubeStep.TRANSLADAR
            self.send_feedback(0.36, 'indo para a frente do cubo')
            #funcao de mover
            time.sleep(1)
            self._pending_reply = 'move'
            self._current_reply = 'move' 
            self._reply_received = True 

        elif self.step == PickCubeStep.TRANSLADAR and self.command_finished():
            self.step = PickCubeStep.AVANCAR
            self.send_feedback(0.45, 'avancando em direcao ao cubo')
            #funcao de mover
            time.sleep(1)
            self._pending_reply = 'move'
            self._current_reply = 'move' 
            self._reply_received = True 

        elif self.step == PickCubeStep.AVANCAR and self.command_finished():
            self.step = PickCubeStep.FECHAR
            self.send_feedback(0.54, 'fechando garra')
            self.send_command('fechar') #mudar para fechar se já não for

        elif self.step == PickCubeStep.FECHAR and self.command_finished():
            self.step = PickCubeStep.RECUAR_CUBO
            self.send_feedback(0.63, 'recuando')
            #funcao de mover
            time.sleep(1)
            self._pending_reply = 'move'
            self._current_reply = 'move' 
            self._reply_received = True 

        elif self.step == PickCubeStep.RECUAR_CUBO and self.command_finished():
            self.step = PickCubeStep.GIRAR_CHASSI
            self.send_feedback(0.72, 'girando para chassi')
            self.send_command(self.slot_to_cmd(args[4]))

        elif self.step == PickCubeStep.GIRAR_CHASSI and self.command_finished():
            self.step = PickCubeStep.VERTICAL_CHASSI
            self.send_feedback(0.81, 'abaixando para altura do chassi')
            self.send_command(self.height_to_cmd('chassi'))

        elif self.step == PickCubeStep.VERTICAL_CHASSI and self.command_finished():
            self.step = PickCubeStep.ABRIR
            self.send_feedback(0.90, 'abrindo garra')
            self.send_command('abrir')

        elif self.step == PickCubeStep.ABRIR and self.command_finished():
            self.step = PickCubeStep.VERTICAL_INICIAL
            self.send_feedback(0.99, 'movendo garra para posicao inicial')
            self.send_command('inicial')

        elif self.step == PickCubeStep.FINISH and self.command_finished():
            self.finish(True, 1.0, 'pick-from-location concluído')
            self.step = 'FINISHED'


def main(args=None):
    rclpy.init(args=args)
    node = PickCubeFromLocationAction()
    node.trigger_configure()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()