import rclpy
from .gripper_primitives import GripperPrimitives
from enum import Enum, auto
import time

class PlaceCubeAtLocationStep(Enum):
    #quando os ultrassonicos detectarem um cobo, meio que tira uma "foto", como a torre e a camera possuem uma distância fixa
    #o cubo detectado sempre estará na mesma posição da camera/shot, logo deve ser analisado aquele apriltag específico para inspencionar
    #se o cubo deve ser pego, ou não, e continuar o sacneamento para próximos cubos
    START = auto()
    TRANSLADAR = auto()
    VERTICAL_CHASSI1 = auto() #pular as próximas duas etapas se o cubo já estiver em modo de "depositar" cubos, já que será realizado pela anterior
    GIRAR_CHASSI1 = auto()
    VERTICAL_CHASSI2 = auto() #inicia daqui se está em modo de depositar
    FECHAR = auto()
    VERTICAL_CHASSI3 = auto()
    GIRAR_AREA = auto()
    VERTICAL_AREA1 = auto()
    ABRIR = auto()
    VERTICAL_AREA2 = auto() 
    GIRAR_CHASSI2 = auto() #ao final dessa ação deve-se analisar se temos outros cubos para depositar, se sim, já deve-se girar a garra para em cima do próximo cubo a ser pedo
    FINISH = auto()


class PlaceCubeAtLocationAction(GripperPrimitives):
    def __init__(self):
        super().__init__('place_at_location') #nome precisa bater com nome da action no pddl domain
        self.step = PlaceCubeAtLocationStep.START

    def do_work(self):
        args = self.current_arguments  # [robot, cube, location, slot] guarda os parâmetros mandados pela ação

        if self.step == PlaceCubeAtLocationStep.START:
            self.step = PlaceCubeAtLocationStep.TRANSLADAR
            self.send_feedback(0.01, 'posicionando em frente ao cubo')
            #incluir aqui funcao que faz o robo recuar, ver questão de feedback para concluir, já que isso é realizado por send_commands(), que publica no tópico da garra que não tange as rodas e sua movimentação
            time.sleep(1)
            self._pending_reply = 'move' #temp
            self._current_reply = 'move' 
            self._reply_received = True 

        elif self.step == PlaceCubeAtLocationStep.TRANSLADAR and self.command_finished():
            self.step = PlaceCubeAtLocationStep.VERTICAL_CHASSI1
            self.send_feedback(0.09, 'levantando garra para girar')
            self.send_command('alt_giro') #mudar comando para frente

        elif self.step == PlaceCubeAtLocationStep.VERTICAL_CHASSI1 and self.command_finished():
            self.step = PlaceCubeAtLocationStep.GIRAR_CHASSI1
            self.send_feedback(0.18, 'girando para slot do cubo')
            self.send_command(self.slot_to_cmd(args[3])) #mudar comando para frente

        elif self.step == PlaceCubeAtLocationStep.GIRAR_CHASSI1 and self.command_finished():
            self.step = PlaceCubeAtLocationStep.VERTICAL_CHASSI2
            self.send_feedback(0.27, 'movendo para altura dos slots')
            self.send_command('inicial')

        elif self.step == PlaceCubeAtLocationStep.VERTICAL_CHASSI2 and self.command_finished():
            self.step = PlaceCubeAtLocationStep.FECHAR
            self.send_feedback(0.36, 'fechando garra')
            self.send_command('fechar') 

        elif self.step == PlaceCubeAtLocationStep.FECHAR and self.command_finished():
            self.step = PlaceCubeAtLocationStep.VERTICAL_CHASSI3
            self.send_feedback(0.45, 'subindo para altura de giro')
            self.send_command('alt_giro')

        elif self.step == PlaceCubeAtLocationStep.VERTICAL_CHASSI3 and self.command_finished():
            self.step = PlaceCubeAtLocationStep.GIRAR_AREA
            self.send_feedback(0.54, 'girando para área')
            self.send_command('frente')

        elif self.step == PlaceCubeAtLocationStep.GIRAR_AREA and self.command_finished():
            self.step = PlaceCubeAtLocationStep.VERTICAL_AREA1
            self.send_feedback(0.63, 'abaixando para altura da área')
            self.send_command(self.height_to_cmd(args[2]))

        elif self.step == PlaceCubeAtLocationStep.VERTICAL_AREA1 and self.command_finished():
            self.step = PlaceCubeAtLocationStep.ABRIR
            self.send_feedback(0.72, 'abrindo garra')
            self.send_command('abrir')

        elif self.step == PlaceCubeAtLocationStep.ABRIR and self.command_finished():
            self.step = PlaceCubeAtLocationStep.VERTICAL_AREA2
            self.send_feedback(0.81, 'subindo para altura de giro')
            self.send_command('abrir')

        elif self.step == PlaceCubeAtLocationStep.VERTICAL_AREA2 and self.command_finished():
            self.step = PlaceCubeAtLocationStep.GIRAR_CHASSI2
            self.send_feedback(0.90, 'girando para posição inicial')
            self.send_command('slot2')

        elif self.step == PlaceCubeAtLocationStep.GIRAR_CHASSI2 and self.command_finished():
            self.step = PlaceCubeAtLocationStep.FINISH
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