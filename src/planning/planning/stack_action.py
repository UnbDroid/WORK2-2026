import rclpy
from .gripper_primitives import GripperPrimitives
from enum import Enum, auto
import time

class StackStep(Enum):
    #quando os ultrassonicos detectarem um cobo, meio que tira uma "foto", como a torre e a camera possuem uma distância fixa
    #o cubo detectado sempre estará na mesma posição da camera/shot, logo deve ser analisado aquele apriltag específico para inspencionar
    #se o cubo deve ser pego, ou não, e continuar o sacneamento para próximos cubos
    START = auto()
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

    def do_work(self):
        args = self.current_arguments  # [robot, top, bottom, location, slot] — REVISAR: confirme a ordem real dos parâmetros de "stack" no domain.pddl

        if self.step == StackStep.START:
            self.step = StackStep.TRANSLADAR
            self.send_feedback(0.02, 'indo para posicao de empilhamento')
            # funcao de mover
            time.sleep(1)
            self._pending_reply = 'move'
            self._current_reply = 'move'
            self._reply_received = True

        elif self.step == StackStep.TRANSLADAR and self.command_finished():
            self.step = StackStep.VERTICAL_CHASSI1
            self.send_feedback(0.08, 'levantando garra para girar ate o chassi')
            self.send_command('alt_giro')

        elif self.step == StackStep.VERTICAL_CHASSI1 and self.command_finished():
            self.step = StackStep.GIRAR_CHASSI1
            self.send_feedback(0.14, 'girando para o slot do cubo')
            self.send_command(self.slot_to_cmd(args[4]))

        elif self.step == StackStep.GIRAR_CHASSI1 and self.command_finished():
            self.step = StackStep.VERTICAL_CHASSI2
            self.send_feedback(0.20, 'abaixando ate a altura do slot')
            self.send_command('inicial')

        elif self.step == StackStep.VERTICAL_CHASSI2 and self.command_finished():
            self.step = StackStep.FECHAR1
            self.send_feedback(0.26, 'fechando garra')
            self.send_command('fechar')

        elif self.step == StackStep.FECHAR1 and self.command_finished():
            self.step = StackStep.VERTICAL_CHASSI3
            self.send_feedback(0.32, 'levantando garra com o cubo')
            self.send_command('alt_giro')

        elif self.step == StackStep.VERTICAL_CHASSI3 and self.command_finished():
            self.step = StackStep.GIRAR_AREA1
            self.send_feedback(0.38, 'girando para a area de empilhamento')
            self.send_command('frente')

        elif self.step == StackStep.GIRAR_AREA1 and self.command_finished():
            self.step = StackStep.STACK1
            self.send_feedback(0.44, 'abaixando para empilhar cubo 1')
            self.send_command(f'empilha:{self.height_to_cmd(args[3])}:bottom')

        elif self.step == StackStep.STACK1 and self.command_finished():
            self.step = StackStep.ABRIR1
            self.send_feedback(0.50, 'abrindo garra')
            self.send_command('abrir')

        elif self.step == StackStep.ABRIR1 and self.command_finished():
            self.step = StackStep.VERTICAL_AREA1
            self.send_feedback(0.56, 'subindo apos soltar o cubo')
            self.send_command('alt_giro')

        elif self.step == StackStep.VERTICAL_AREA1 and self.command_finished():
            self.step = StackStep.GIRAR_CHASSI2
            self.send_feedback(0.62, 'girando para pegar cubo 2')
            self.send_command(self.slot_to_cmd(args[5]))  

        elif self.step == StackStep.GIRAR_CHASSI2 and self.command_finished():
            self.step = StackStep.VERTICAL_CHASSI4
            self.send_feedback(0.68, 'abaixando ate a altura do proximo slot')
            self.send_command('inicial')

        elif self.step == StackStep.VERTICAL_CHASSI4 and self.command_finished():
            self.step = StackStep.FECHAR2
            self.send_feedback(0.74, 'fechando garra')
            self.send_command('fechar')

        elif self.step == StackStep.FECHAR2 and self.command_finished():
            self.step = StackStep.VERTICAL_CHASSI5
            self.send_feedback(0.80, 'levantando garra com o segundo cubo')
            self.send_command('alt_giro')

        elif self.step == StackStep.VERTICAL_CHASSI5 and self.command_finished():
            self.step = StackStep.GIRAR_AREA2
            self.send_feedback(0.86, 'girando para a area de empilhamento novamente')
            self.send_command('frente')

        elif self.step == StackStep.GIRAR_AREA2 and self.command_finished():
            self.step = StackStep.STACK2
            self.send_feedback(0.90, 'abaixando para empilhar o cubo 2')
            self.send_command(f'empilha:{self.height_to_cmd(args[3])}:top')  

        elif self.step == StackStep.STACK2 and self.command_finished():
            self.step = StackStep.ABRIR2
            self.send_feedback(0.94, 'abrindo garra')
            self.send_command('abrir')

        elif self.step == StackStep.ABRIR2 and self.command_finished():
            self.step = StackStep.VERTICAL_AREA2
            self.send_feedback(0.97, 'subindo para altura original')
            self.send_command('alt_giro')

        elif self.step == StackStep.VERTICAL_AREA2 and self.command_finished():
            self.step = StackStep.GIRAR_CHASSI3
            self.send_feedback(0.99, 'girando garra de volta a posicao neutra')
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