import rclpy
from .gripper_primitives import GripperPrimitives
from enum import Enum, auto
from std_msgs.msg import String
import time


class PickCubeStep(Enum):
    START = auto()
    AJUSTAR = auto()
    TRANSLADAR = auto()
    ABRIR1 = auto()
    VERTICAL_AREA1 = auto()
    GIRAR_AREA = auto()
    VERTICAL_AREA2 = auto()
    FECHAR = auto()
    VERTICAL_CHASSI1 = auto()
    GIRAR_CHASSI = auto()
    VERTICAL_CHASSI2 = auto()
    ABRIR2 = auto()


class PickCubeFromLocationAction(GripperPrimitives):

    def __init__(self):
        super().__init__('pick_from_location')

        self.step = PickCubeStep.START
        self.target_location = None
        self.angulo = None

        # NOVO: indica que a coleta deve ser pulada
        self.impossivel = False

        # NOVO: recebe mensagens do tópico da garra
        self.sub_impossivel = self.create_subscription(
            String,
            '/topico_garra',
            self.impossivel_callback,
            10
        )

    # =====================================
    # NOVO: RECEBER IMPOSSIVEL
    # =====================================

    def impossivel_callback(self, msg):

        if msg.data == 'impossivel':

            if self.step != PickCubeStep.START:

                self.get_logger().warn(
                    'Recebido impossivel! '
                    'Pulando coleta do cubo.'
                )

                self.impossivel = True

    # =====================================
    # EXECUÇÃO DA AÇÃO
    # =====================================

    def do_work(self):

        args = self.current_arguments

        # =====================================
        # NOVO: PULAR COLETA
        # =====================================

        if self.impossivel:

            self.get_logger().warn(
                'Coleta ignorada devido à proximidade da parede.'
            )

            self.impossivel = False
            self.step = PickCubeStep.START

            self.finish(
                True,
                1.0,
                'pick-from-location concluído'
            )

            return

        # =====================================
        # START
        # =====================================

        if self.step == PickCubeStep.START:

            self.impossivel = False

            self.step = PickCubeStep.AJUSTAR

            self.target_location = args[2]
            self.angulo = self.get_location_angle(
                self.target_location
            )

            self.send_feedback(
                0.01,
                f'Ajustando distância e angulação '
                f'de {self.angulo} graus da mesa'
            )

            time.sleep(1)

            self.send_command(
                f'ajustar:{self.height_to_cmd(args[2])}'
            )

        # =====================================
        # AJUSTAR
        # =====================================

        elif (
            self.step == PickCubeStep.AJUSTAR
            and self.command_finished()
        ):

            self.step = PickCubeStep.TRANSLADAR

            self.send_feedback(
                0.06,
                'Posicionando em frente ao cubo'
            )

            time.sleep(1)

            self.send_command(
                f'transladar:{args[1]}:'
                f'{self.height_to_cmd(args[2])}:{args[2]}'
            )

        elif (
            self.step == PickCubeStep.TRANSLADAR
            and self.command_finished()
        ):

            self.step = PickCubeStep.ABRIR1

            self.send_feedback(
                0.10,
                'Abrindo garra'
            )

            self.send_command('abrir')

        # =====================================
        # TRANSLADAR
        # =====================================

        elif (
            self.step == PickCubeStep.ABRIR1
            and self.command_finished()
        ):

            self.step = PickCubeStep.VERTICAL_AREA1

            if self.target_location in ['loc4', 'loc10']:
                self.send_feedback(
                    0.11,
                    'Levantando garra para girar pela esquerda'
                )

                self.send_command('alt_giro_esquerda')
            else:
                self.send_feedback(
                    0.11,
                    'Levantando garra para girar pela direita'
                )

                self.send_command('alt_giro')

        # =====================================
        # VERTICAL AREA 1
        # =====================================

        elif (
            self.step == PickCubeStep.VERTICAL_AREA1
            and self.command_finished()
        ):

            self.step = PickCubeStep.GIRAR_AREA

            elif self.target_location in ['loc4', 'loc10']:
                self.step = PlaceCubeAtLocationStep.GIRAR_AREA
                self.send_command('frente_esq')

            else:

                self.send_feedback(
                    0.22,
                    'Girando pegar cubo'
                )

                self.send_command('frente')

        # =====================================
        # GIRAR AREA
        # =====================================

        elif (
            self.step == PickCubeStep.GIRAR_AREA
            and self.command_finished()
        ):

            self.step = PickCubeStep.VERTICAL_AREA2

            self.send_feedback(
                0.33,
                f'Movendo para altura de {args[2]}'
            )

            self.send_command(
                self.height_to_cmd(args[2])
            )

        # =====================================
        # VERTICAL AREA 2
        # =====================================

        elif (
            self.step == PickCubeStep.VERTICAL_AREA2
            and self.command_finished()
        ):

            self.step = PickCubeStep.FECHAR

            self.send_feedback(
                0.44,
                'Fechando garra'
            )

            self.send_command('fechar')

        # =====================================
        # FECHAR
        # =====================================

        elif (
            self.step == PickCubeStep.FECHAR
            and self.command_finished()
        ):

            self.step = PickCubeStep.VERTICAL_CHASSI1

            if self.target_location in ['loc4', 'loc10']:
                self.send_feedback(
                    0.55,
                    'Levantando garra com cubo para girar pela esquerda'
                )

                self.send_command('alt_giro_esquerda')
            else:

                self.send_feedback(
                    0.55,
                    'Subindo com cubo para altura de giro pela direita'
                )

                self.send_command('alt_giro')

        # =====================================
        # VERTICAL CHASSI 1
        # =====================================

        elif (
            self.step == PickCubeStep.VERTICAL_CHASSI1
            and self.command_finished()
        ):

            self.step = PickCubeStep.GIRAR_CHASSI

            self.send_feedback(
                0.66,
                f'Girando para {args[3]}'
            )

            self.send_command(
                self.slot_to_cmd(args[3])
            )

        # =====================================
        # GIRAR CHASSI
        # =====================================

        elif (
            self.step == PickCubeStep.GIRAR_CHASSI
            and self.command_finished()
        ):

            self.step = PickCubeStep.VERTICAL_CHASSI2

            self.send_feedback(
                0.77,
                'Abaixando para altura dos slots'
            )

            self.send_command('inicial')

        # =====================================
        # VERTICAL CHASSI 2
        # =====================================

        elif (
            self.step == PickCubeStep.VERTICAL_CHASSI2
            and self.command_finished()
        ):

            self.step = PickCubeStep.ABRIR2

            self.send_feedback(
                0.88,
                'Abrindo garra'
            )

            self.send_command('abrir')

        # =====================================
        # FINALIZAÇÃO NORMAL
        # =====================================

        elif (
            self.step == PickCubeStep.ABRIR2
            and self.command_finished()
        ):

            self.step = PickCubeStep.START

            self.finish(
                True,
                1.0,
                'pick-from-location concluído'
            )


def main(args=None):

    rclpy.init(args=args)

    node = PickCubeFromLocationAction()

    node.trigger_configure()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()