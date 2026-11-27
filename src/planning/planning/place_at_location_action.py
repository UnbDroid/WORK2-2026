import rclpy
from .gripper_primitives import GripperPrimitives
from enum import Enum, auto
import time

DIST_TRANSLACAO_M = -0.3   # + = esquerda, - = direita mudei

class PlaceCubeAtLocationStep(Enum):
    #quando os ultrassonicos detectarem um cobo, meio que tira uma "foto", como a torre e a camera possuem uma distância fixa
    #o cubo detectado sempre estará na mesma posição da camera/shot, logo deve ser analisado aquele apriltag específico para inspencionar
    #se o cubo deve ser pego, ou não, e continuar o sacneamento para próximos cubos
    START = auto()
    AJUSTAR1 = auto() #mudei
    TRANSLADAR = auto() #mudei
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
    AJUSTAR2 = auto()


class PlaceCubeAtLocationAction(GripperPrimitives):
    def __init__(self):
        super().__init__('place_at_location') #nome precisa bater com nome da action no pddl domain
        self.step = PlaceCubeAtLocationStep.START
        self.target_location = None
        self.angulo = None

    def do_work(self):
        args = self.current_arguments  # [robot, cube, location, slot, position] guarda os parâmetros mandados pela ação

        if self.step == PlaceCubeAtLocationStep.START:
            self.step = PlaceCubeAtLocationStep.AJUSTAR1
            self.target_location = args[2]
            self.angulo = self.get_location_angle(self.target_location)
            self.send_feedback(0.01, f'Ajustando distância de 20 cm e angulação de {self.angulo} graus da mesa')
            time.sleep(1)  # delay antes de publicar o comando
            self.send_command(f'ajustar:{self.height_to_cmd(args[2])}') 

        elif self.step == PlaceCubeAtLocationStep.AJUSTAR1 and self.command_finished():
            self.step = PlaceCubeAtLocationStep.TRANSLADAR
            self.send_feedback(0.06, f'Transladando {DIST_TRANSLACAO_M} m de lado')
            self.start_translation(DIST_TRANSLACAO_M, velocidade=0.2)

        elif self.step == PlaceCubeAtLocationStep.TRANSLADAR and self.translation_finished():
            self.step = PlaceCubeAtLocationStep.VERTICAL_CHASSI1
            self.send_feedback(0.09, 'Levantando garra para girar')
            self.send_command('alt_giro')

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
            if args[2] in ['locshelf2', 'locshelf4', 'locshelf6', 'locshelf8']:
                self.send_command('shelf')
                self.send_feedback(0.45, 'Subindo para altura da shelf')
            elif self.target_location in ['loc4', 'loc10']:
                self.send_command('alt_giro_esquerda')
                self.send_feedback(0.45, 'Subindo para altura de giro à esquerda') 
            else:   
                self.send_command('alt_giro') #mudar comando para frente
                self.send_feedback(0.45, 'Subindo para altura de giro')


        elif self.step == PlaceCubeAtLocationStep.VERTICAL_CHASSI3 and self.command_finished():
            if args[2] == 'locshelf2': #mudar
                self.step = PlaceCubeAtLocationStep.VERTICAL_AREA1
                self.send_command(self.slot_to_cmd(args[4]))
            elif self.target_location in ['loc4', 'loc10']:
                self.step = PlaceCubeAtLocationStep.GIRAR_AREA
                self.send_command('frente_esq')
            else:
                self.step = PlaceCubeAtLocationStep.GIRAR_AREA
                self.send_command(self.slot_to_cmd(args[4]))
            self.send_feedback(0.54, f'Girando para posição {args[4]}')

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
            self.step = PlaceCubeAtLocationStep.AJUSTAR2
            self.send_feedback(0.95, 'AJUSTE FINAL')
            self.send_command(f'ajustar:{self.height_to_cmd(args[2])}') 
            time.sleep(1)

        elif self.step == PlaceCubeAtLocationStep.AJUSTAR2 and self.command_finished():
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