import rclpy
from .gripper_primitives import GripperPrimitives
from enum import Enum, auto
import time

class PlaceCubeAtLocationStep(Enum):
    #quando os ultrassonicos detectarem um cobo, meio que tira uma "foto", como a torre e a camera possuem uma distância fixa
    #o cubo detectado sempre estará na mesma posição da camera/shot, logo deve ser analisado aquele apriltag específico para inspencionar
    #se o cubo deve ser pego, ou não, e continuar o sacneamento para próximos cubos
    START = auto()
    AJUSTAR = auto()
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


class PlaceCubeAtLocationAction(GripperPrimitives):
    def __init__(self):
        super().__init__('place_at_location') #nome precisa bater com nome da action no pddl domain
        self.step = PlaceCubeAtLocationStep.START
        self.target_location = None
        self.angulo = None

    def do_work(self):
        args = self.current_arguments  # [robot, cube, location, slot, position] guarda os parâmetros mandados pela ação

        if self.step == PlaceCubeAtLocationStep.START:
            self.step = PlaceCubeAtLocationStep.AJUSTAR
            self.target_location = args[2]
            self.angulo = self.get_location_angle(self.target_location)
            self.send_feedback(0.01, f'Ajustando distância de 20 cm e angulação de {self.angulo} graus da mesa')
            time.sleep(1)  # delay antes de publicar o comando
            self.send_command(f'ajustar:{self.height_to_cmd(args[2])}') 
        
        elif self.step == PlaceCubeAtLocationStep.AJUSTAR and self.command_finished():
            self.step = PlaceCubeAtLocationStep.VERTICAL_CHASSI1
            self.send_feedback(0.09, 'Levantando garra para girar')
            self.send_command('alt_giro') #mudar comando para frente

        elif self.step == PlaceCubeAtLocationStep.VERTICAL_CHASSI1 and self.command_finished():
            self.step = PlaceCubeAtLocationStep.GIRAR_CHASSI1
            self.send_feedback(0.18, f'Girando para {args[3]}')
            self.send_command(self.slot_to_cmd(args[3])) #mudar comando para frente

        elif self.step == PlaceCubeAtLocationStep.GIRAR_CHASSI1 and self.command_finished():
            self.step = PlaceCubeAtLocationStep.VERTICAL_CHASSI2
            self.send_feedback(0.27, 'Movendo para altura dos slots')
            self.send_command('inicial')

        elif self.step == PlaceCubeAtLocationStep.VERTICAL_CHASSI2 and self.command_finished():
            self.step = PlaceCubeAtLocationStep.FECHAR
            self.send_feedback(0.36, 'Fechando garra')
            self.send_command('fechar') 

        elif self.step == PlaceCubeAtLocationStep.FECHAR and self.command_finished():
            self.step = PlaceCubeAtLocationStep.VERTICAL_CHASSI3
            if args[2] == 'locshelf2':
                self.send_command('shelf')
                self.send_feedback(0.45, 'Subindo para altura da shelf')
            else:   
                self.send_command('alt_giro') #mudar comando para frente
                self.send_feedback(0.45, 'Subindo para altura de giro')


        elif self.step == PlaceCubeAtLocationStep.VERTICAL_CHASSI3 and self.command_finished():
            if args[2] == 'locshelf2':
                self.step = PlaceCubeAtLocationStep.VERTICAL_AREA1
            else:
                self.step = PlaceCubeAtLocationStep.GIRAR_AREA
            self.send_feedback(0.54, f'Girando para posição {args[4]}')
            self.send_command(self.slot_to_cmd(args[4])) #manda para diferentes posições em vez de apenas para frente

        elif self.step == PlaceCubeAtLocationStep.GIRAR_AREA and self.command_finished():
            self.step = PlaceCubeAtLocationStep.VERTICAL_AREA1
            self.send_feedback(0.63, f'Abaixando para altura de {args[2]}')
            self.send_command(self.height_to_cmd(args[2]))

        elif self.step == PlaceCubeAtLocationStep.VERTICAL_AREA1 and self.command_finished():
            if args[2] == 'locshelf2':
               self.step = PlaceCubeAtLocationStep.VERTICAL_AREA2
            else:
                self.step = PlaceCubeAtLocationStep.ABRIR
            self.send_feedback(0.72, 'Abrindo garra')
            self.send_command('abrir')

        elif self.step == PlaceCubeAtLocationStep.ABRIR and self.command_finished():
            self.step = PlaceCubeAtLocationStep.VERTICAL_AREA2
            self.send_feedback(0.81, 'Subindo para altura de giro')
            self.send_command('alt_giro')

        elif self.step == PlaceCubeAtLocationStep.VERTICAL_AREA2 and self.command_finished():
            self.step = PlaceCubeAtLocationStep.GIRAR_CHASSI2
            self.send_feedback(0.90, 'Girando para posição inicial')
            self.send_command('slot2')

        elif self.step == PlaceCubeAtLocationStep.GIRAR_CHASSI2 and self.command_finished():
            self.step = PlaceCubeAtLocationStep.START #cria um loop para próxima ação da garra
            self.finish(True, 1.0, 'pick-from-location concluído')     


def main(args=None):
    rclpy.init(args=args)
    node = PlaceCubeAtLocationAction()
    node.trigger_configure()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()