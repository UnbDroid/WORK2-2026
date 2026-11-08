import rclpy
from .gripper_primitives import GripperPrimitives
from enum import Enum, auto
import time

class PickCubeStep(Enum):
    #quando os ultrassonicos detectarem um cobo, meio que tira uma "foto", como a torre e a camera possuem uma distância fixa
    #o cubo detectado sempre estará na mesma posição da camera/shot, logo deve ser analisado aquele apriltag específico para inspencionar
    #se o cubo deve ser pego, ou não, e continuar o sacneamento para próximos cubos
    START = auto()
    AJUSTAR = auto()
    TRANSLADAR = auto()
    VERTICAL_AREA1 = auto()
    GIRAR_AREA = auto() #gira garra para zona de pegar cubos
    VERTICAL_AREA2 = auto() #podem ser simultâneos girar e vertical a partir de um determinado ângulo
    FECHAR = auto()
    VERTICAL_CHASSI1 = auto() #vertical para altura inicial
    GIRAR_CHASSI = auto() #podem ser simultaneos girar e vertical_chassi2 SE ALTURA FOR MAIOR QUE DO ROBO, a
    VERTICAL_CHASSI2 = auto()  #vertical para altura dos slots
    ABRIR = auto()


class PickCubeFromLocationAction(GripperPrimitives):
    def __init__(self):
        super().__init__('pick_from_location') #nome precisa bater com nome da action no pddl domain
        self.step = PickCubeStep.START
        self.target_location = None
        self.angulo = None

    def do_work(self):
        args = self.current_arguments  # [robot, cube, location, slot] guarda os parâmetros mandados pela ação

        if self.step == PickCubeStep.START:
            self.step = PickCubeStep.AJUSTAR
            self.target_location = args[2]
            self.angulo = self.get_location_angle(self.target_location)
            self.send_feedback(0.01, f'Ajustando distância de 20 cm e angulação de {self.angulo} graus da mesa')
            self.send_command(f'ajustar:{self.angulo}') 
        
        elif self.step == PickCubeStep.AJUSTAR and self.command_finished():
            self.step = PickCubeStep.TRANSLADAR
            self.send_feedback(0.06, 'Posicionando em frente ao cubo')
            self.send_command(f'transladar:{args[1]}:{self.height_to_cmd(args[2])}') #indica que comando de transladar deve iniciar

        elif self.step == PickCubeStep.TRANSLADAR and self.command_finished():
            self.step = PickCubeStep.VERTICAL_AREA1
            self.send_feedback(0.11, 'Levantando garra para girar')
            self.send_command('alt_giro') #mudar comando para frente

        elif self.step == PickCubeStep.VERTICAL_AREA1 and self.command_finished():
            self.step = PickCubeStep.GIRAR_AREA
            self.send_feedback(0.22, 'Girando pegar cubo')
            self.send_command('frente') #mudar comando para frente

        elif self.step == PickCubeStep.GIRAR_AREA and self.command_finished():
            self.step = PickCubeStep.VERTICAL_AREA2
            self.send_feedback(0.33, f'Movendo para altura de {args[2]}')
            self.send_command(self.height_to_cmd(args[2]))

        elif self.step == PickCubeStep.VERTICAL_AREA2 and self.command_finished():
            self.step = PickCubeStep.FECHAR
            self.send_feedback(0.44, 'Fechando garra')
            self.send_command('fechar') 

        elif self.step == PickCubeStep.FECHAR and self.command_finished():
            self.step = PickCubeStep.VERTICAL_CHASSI1
            self.send_feedback(0.55, 'Subindo com cubo para altura de giro')
            self.send_command('alt_giro')

        elif self.step == PickCubeStep.VERTICAL_CHASSI1 and self.command_finished():
            self.step = PickCubeStep.GIRAR_CHASSI
            self.send_feedback(0.66, f'Girando para {args[3]}')
            self.send_command(self.slot_to_cmd(args[3]))

        elif self.step == PickCubeStep.GIRAR_CHASSI and self.command_finished():
            self.step = PickCubeStep.VERTICAL_CHASSI2
            self.send_feedback(0.77, 'Abaixando para altura dos slots')
            self.send_command('inicial')

        elif self.step == PickCubeStep.VERTICAL_CHASSI2 and self.command_finished():
            self.step = PickCubeStep.ABRIR
            self.send_feedback(0.88, 'Abrindo garra')
            self.send_command('abrir')

        elif self.step == PickCubeStep.ABRIR and self.command_finished():
            self.step = PickCubeStep.START
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