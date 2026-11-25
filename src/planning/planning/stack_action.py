import rclpy
from .gripper_primitives import GripperPrimitives
from enum import Enum, auto
import time

class StackStep(Enum):
    #quando os ultrassonicos detectarem um cobo, meio que tira uma "foto", como a torre e a camera possuem uma distância fixa
    #o cubo detectado sempre estará na mesma posição da camera/shot, logo deve ser analisado aquele apriltag específico para inspencionar
    #se o cubo deve ser pego, ou não, e continuar o sacneamento para próximos cubos
    START = auto()
    AJUSTAR = auto()
    TRANSLADAR = auto()
    VERTICAL_CHASSI1 = auto()  # pular as próximas duas etapas se o cubo já estiver em modo de "depositar" cubos, já que será realizado pela anterior
    GIRAR_CHASSI1 = auto()
    VERTICAL_CHASSI2 = auto()  # inicia daqui se está em modo de depositar
    FECHAR1 = auto()
    VERTICAL_CHASSI3 = auto()
    GIRAR_AREA1 = auto()
    STACK1 = auto()
    ABRIR1 = auto()
    VERTICAL_AREA1 = auto()
    GIRAR_CHASSI2 = auto()  # ao final dessa ação deve-se analisar se temos outros cubos para depositar, se sim, já deve-se girar a garra para em cima do próximo cubo a ser pego
    VERTICAL_CHASSI4 = auto()
    FECHAR2 = auto()
    VERTICAL_CHASSI5 = auto()
    GIRAR_AREA2 = auto()
    STACK2 = auto()
    ABRIR2 = auto()
    VERTICAL_AREA2 = auto()
    GIRAR_CHASSI3 = auto()
    FINISH = auto()


class StackAction(GripperPrimitives):
    def __init__(self):
        super().__init__('stack')  # nome precisa bater com nome da action no pddl domain
        self.step = StackStep.START
        self.target_location = None
        self.angulo = None

    def do_work(self):
        args = self.current_arguments  # [robot, bottom, top, location, bottomslot, topslot, position, tag]

        if self.step == StackStep.START:
            self.step = StackStep.AJUSTAR
            self.target_location = args[3]
            self.angulo = self.get_location_angle(self.target_location)
            self.send_feedback(0.01, f'Ajustando distância de 20 cm e angulação de {self.angulo} graus da mesa')
            self.send_command(f'ajustar:{self.height_to_cmd(args[3])}') 

        elif self.step == StackStep.AJUSTAR and self.command_finished():
            self.step = StackStep.TRANSLADAR
            self.send_feedback(0.04, 'Procurando tag 200 para empilhar')
            self.send_command(f'transladar:cube7:{self.height_to_cmd(args[3])}:{args[3]}')

        elif self.step == StackStep.TRANSLADAR and self.command_finished():
            self.step = StackStep.VERTICAL_CHASSI1
            self.send_feedback(0.08, 'Levantando garra para girar')
            self.send_command('alt_giro')

        elif self.step == StackStep.VERTICAL_CHASSI1 and self.command_finished():
            self.step = StackStep.GIRAR_CHASSI1
            self.send_feedback(0.14, f'Girando para {args[4]} do cubo inferior')
            self.send_command(self.slot_to_cmd(args[4]))

        elif self.step == StackStep.GIRAR_CHASSI1 and self.command_finished():
            self.step = StackStep.VERTICAL_CHASSI2
            self.send_feedback(0.20, 'Abaixando ate a altura do slot')
            self.send_command('inicial')

        elif self.step == StackStep.VERTICAL_CHASSI2 and self.command_finished():
            self.step = StackStep.FECHAR1
            self.send_feedback(0.26, 'Fechando garra')
            self.send_command('fechar')

        elif self.step == StackStep.FECHAR1 and self.command_finished():
            self.step = StackStep.VERTICAL_CHASSI3
            if self.target_location in ['loc4', 'loc10']:
                self.send_feedback(0.32, f'Levantando garra com {args[1]} para girar pela esquerda')
                self.send_command('alt_giro_esquerda')
            else:
                self.send_feedback(0.32, f'Levantando garra com {args[1]} para girar direita')
                self.send_command('alt_giro')

        elif self.step == StackStep.VERTICAL_CHASSI3 and self.command_finished():
            self.step = StackStep.GIRAR_AREA1
            self.send_feedback(0.38, f'Girando para posição {args[6]}')
            self.send_command(self.slot_to_cmd(args[6]))

        elif self.step == StackStep.GIRAR_AREA1 and self.command_finished(): #para aqui, acho que não está mandando o comando para a main corretamente
            self.step = StackStep.STACK1
            self.send_feedback(0.44, f'Abaixando para empilhar cubo debaixo: {args[1]}')
            self.send_command(f'empilha:{self.height_to_cmd(args[3])}:bottom')

        elif self.step == StackStep.STACK1 and self.command_finished():
            self.step = StackStep.ABRIR1
            self.send_feedback(0.50, 'Abrindo garra')
            self.send_command('abrir')

        elif self.step == StackStep.ABRIR1 and self.command_finished():
            self.step = StackStep.VERTICAL_AREA1
            if self.target_location in ['loc4', 'loc10']:
                self.send_feedback(0.56, f'Levantando para girar pela esquerda')
                self.send_command('alt_giro_esquerda')
            else:
                self.send_feedback(0.56, f'Levantando para girar direita')
                self.send_command('alt_giro')


        elif self.step == StackStep.VERTICAL_AREA1 and self.command_finished():
            self.step = StackStep.GIRAR_CHASSI2
            self.send_feedback(0.62, f'Girando para {args[5]}')
            self.send_command(self.slot_to_cmd(args[5]))  

        elif self.step == StackStep.GIRAR_CHASSI2 and self.command_finished():
            self.step = StackStep.VERTICAL_CHASSI4
            self.send_feedback(0.68, 'Abaixando ate a altura dos slots')
            self.send_command('inicial')

        elif self.step == StackStep.VERTICAL_CHASSI4 and self.command_finished():
            self.step = StackStep.FECHAR2
            self.send_feedback(0.74, 'Fechando garra')
            self.send_command('fechar')

        elif self.step == StackStep.FECHAR2 and self.command_finished():
            self.step = StackStep.VERTICAL_CHASSI5
            if self.target_location in ['loc4', 'loc10']:
                self.send_feedback(0.80, f'Levantando garra com {args[3]} para girar pela esquerda')
                self.send_command('alt_giro_esquerda')
            else:
                self.send_feedback(0.80, f'Levantando garra com {args[3]} para girar direita')
                self.send_command('alt_giro')

        elif self.step == StackStep.VERTICAL_CHASSI5 and self.command_finished():
            self.step = StackStep.GIRAR_AREA2
            self.send_feedback(0.86, f'Girando para posição {args[6]}')
            self.send_command(self.slot_to_cmd(args[6]))

        elif self.step == StackStep.GIRAR_AREA2 and self.command_finished():
            self.step = StackStep.STACK2
            self.send_feedback(0.90, f'Abaixando para empilhar cubo de cima: {args[2]}')
            self.send_command(f'empilha:{self.height_to_cmd(args[3])}:top')  

        elif self.step == StackStep.STACK2 and self.command_finished():
            self.step = StackStep.ABRIR2
            self.send_feedback(0.94, 'Abrindo garra')
            self.send_command('abrir')

        elif self.step == StackStep.ABRIR2 and self.command_finished():
            self.step = StackStep.VERTICAL_AREA2
            if self.target_location in ['loc4', 'loc10']:
                self.send_feedback(0.32, f'Levantando garra para girar pela esquerda')
                self.send_command('alt_giro_esquerda')
            else:
                self.send_feedback(0.32, f'Levantando garra para girar direita')
                self.send_command('alt_giro')

        elif self.step == StackStep.VERTICAL_AREA2 and self.command_finished():
            self.step = StackStep.GIRAR_CHASSI3
            self.send_feedback(0.99, 'Girando garra de volta a posicao neutra')
            self.send_command('slot2')  

        elif self.step == StackStep.GIRAR_CHASSI3 and self.command_finished():
            self.step = StackStep.FINISH
            self.finish(True, 1.0, 'stack concluído')


def main(args=None):
    rclpy.init(args=args)
    node = StackAction()
    node.trigger_configure()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()