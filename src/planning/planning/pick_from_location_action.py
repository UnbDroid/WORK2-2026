import rclpy
from .gripper_primitives import GripperPrimitives
from enum import Enum, auto
import time

class PickCubeStep(Enum):
    #quando os ultrassonicos detectarem um cobo, meio que tira uma "foto", como a torre e a camera possuem uma distância fixa
    #o cubo detectado sempre estará na mesma posição da camera/shot, logo deve ser analisado aquele apriltag específico para inspencionar
    #se o cubo deve ser pego, ou não, e continuar o sacneamento para próximos cubos
    START = auto()
    TRANSLADAR = auto()
    VERTICAL_AREA1 = auto()
    GIRAR_AREA = auto() #gira garra para zona de pegar cubos
    VERTICAL_AREA2 = auto() #podem ser simultâneos girar e vertical a partir de um determinado ângulo
    FECHAR = auto()
    VERTICAL_CHASSI1 = auto() #vertical para altura inicial
    GIRAR_CHASSI = auto() #podem ser simultaneos girar e vertical_chassi2 SE ALTURA FOR MAIOR QUE DO ROBO, a
    VERTICAL_CHASSI2 = auto()  #vertical para altura dos slots
    ABRIR = auto()
    FINISH = auto()


class PickCubeFromLocationAction(GripperPrimitives):
    def __init__(self):
        super().__init__('pick_from_location') #nome precisa bater com nome da action no pddl domain
        self.step = PickCubeStep.START

    def do_work(self):
        args = self.current_arguments  # [robot, cube, location, slot] guarda os parâmetros mandados pela ação

        if self.step == PickCubeStep.START:
            self.step = PickCubeStep.TRANSLADAR
            self.send_feedback(0.01, 'posicionando em frente ao cubo')
            #incluir aqui funcao que faz o robo recuar, ver questão de feedback para concluir, já que isso é realizado por send_commands(), que publica no tópico da garra que não tange as rodas e sua movimentação
            time.sleep(1)
            self._pending_reply = 'move' #temp
            self._current_reply = 'move' 
            self._reply_received = True 

        elif self.step == PickCubeStep.TRANSLADAR and self.command_finished():
            self.step = PickCubeStep.VERTICAL_AREA1
            self.send_feedback(0.11, 'levantando garra para girar')
            self.send_command('alt_giro') #mudar comando para frente

        elif self.step == PickCubeStep.VERTICAL_AREA1 and self.command_finished():
            self.step = PickCubeStep.GIRAR_AREA
            self.send_feedback(0.22, 'girando para area de cubo')
            self.send_command('frente') #mudar comando para frente

        elif self.step == PickCubeStep.GIRAR_AREA and self.command_finished():
            self.step = PickCubeStep.VERTICAL_AREA2
            self.send_feedback(0.33, 'movendo para altura da mesa')
            self.send_command(self.height_to_cmd(args[2]))

        elif self.step == PickCubeStep.VERTICAL_AREA2 and self.command_finished():
            self.step = PickCubeStep.FECHAR
            self.send_feedback(0.44, 'fechando garra')
            self.send_command('fechar') #mudar para fechar se já não for

        elif self.step == PickCubeStep.FECHAR and self.command_finished():
            self.step = PickCubeStep.VERTICAL_CHASSI1
            self.send_feedback(0.55, 'subindo para altura de giro')
            self.send_command('alt_giro')

        elif self.step == PickCubeStep.VERTICAL_CHASSI1 and self.command_finished():
            self.step = PickCubeStep.GIRAR_CHASSI
            self.send_feedback(0.66, 'girando para chassi')
            self.send_command(self.slot_to_cmd(args[3]))

        elif self.step == PickCubeStep.GIRAR_CHASSI and self.command_finished():
            self.step = PickCubeStep.VERTICAL_CHASSI2
            self.send_feedback(0.77, 'abaixando para altura dos slots')
            self.send_command('inicial')

        elif self.step == PickCubeStep.VERTICAL_CHASSI2 and self.command_finished():
            self.step = PickCubeStep.ABRIR
            self.send_feedback(0.88, 'abrindo garra')
            self.send_command('abrir')

        elif self.step == PickCubeStep.ABRIR and self.command_finished():
            self.step = PickCubeStep.FINISH
            self.finish(True, 1.0, 'pick-from-location concluído')     


def main(args=None):
    rclpy.init(args=args)
    node = PickCubeFromLocationAction()
    node.trigger_configure()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()