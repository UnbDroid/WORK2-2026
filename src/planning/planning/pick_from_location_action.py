import rclpy
from .gripper_primitives import GripperPrimitives
from enum import Enum, auto
import time

class PickCubeStep(Enum):
    #quando os ultrassonicos detectarem um cobo, meio que tira uma "foto", como a torre e a camera possuem uma distância fixa
    #o cubo detectado sempre estará na mesma posição da camera/shot, logo deve ser analisado aquele apriltag específico para inspencionar
    #se o cubo deve ser pego, ou não, e continuar o sacneamento para próximos cubos
    START = auto()
    RECUAR_INICIAL = auto()
    VERTICAL_AREA1 = auto()
    GIRAR_VERTICAL1 = auto()
    TRANSLADAR = auto()
    AVANCAR = auto()
    FECHAR = auto()
    RECUAR_CUBO = auto()
    GIRAR_VERTICAL2 = auto()
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
            time.sleep(0.25)
            self.step = PickCubeStep.RECUAR_INICIAL
            self.send_feedback(0.01, 'recuando')
            #incluir aqui funcao que faz o robo recuar, ver questão de feedback para concluir, já que isso é realizado por send_commands(), que publica no tópico da garra que não tange as rodas e sua movimentação
            time.sleep(1)
            self._pending_reply = 'move' #temp
            self._current_reply = 'move' 
            self._reply_received = True 

        elif self.step == PickCubeStep.RECUAR_INICIAL and self.command_finished():
            time.sleep(0.25)
            self.step = PickCubeStep.VERTICAL_AREA1
            self.send_feedback(0.10, 'levantando garra para girar')
            self.send_command('alt_giro_bloq') #mudar comando para frente
        
        elif self.step == PickCubeStep.VERTICAL_AREA1 and self.command_finished():
            time.sleep(0.25)
            self.step = PickCubeStep.GIRAR_VERTICAL1
            self.send_feedback(0.20, 'girando e movendo altura simultaneamente')
            self.send_command(f'COMBO:frente:{self.height_to_cmd(args[2])}')

        elif self.step == PickCubeStep.GIRAR_VERTICAL1 and self.command_finished():
            time.sleep(0.25)
            self.step = PickCubeStep.TRANSLADAR
            self.send_feedback(0.30, 'indo para a frente do cubo')
            #funcao de mover
            time.sleep(1)
            self._pending_reply = 'move'
            self._current_reply = 'move' 
            self._reply_received = True 

        elif self.step == PickCubeStep.TRANSLADAR and self.command_finished():
            time.sleep(0.25)
            self.step = PickCubeStep.AVANCAR
            self.send_feedback(0.40, 'avancando em direcao ao cubo')
            #funcao de mover
            time.sleep(1)
            self._pending_reply = 'move'
            self._current_reply = 'move' 
            self._reply_received = True 

        elif self.step == PickCubeStep.AVANCAR and self.command_finished():
            time.sleep(0.25)
            self.step = PickCubeStep.FECHAR
            self.send_feedback(0.50, 'fechando garra')
            self.send_command('fechar') #mudar para fechar se já não for

        elif self.step == PickCubeStep.FECHAR and self.command_finished():
            time.sleep(0.25)
            self.step = PickCubeStep.RECUAR_CUBO
            self.send_feedback(0.60, 'recuando')
            #funcao de mover
            time.sleep(1)
            self._pending_reply = 'move'
            self._current_reply = 'move' 
            self._reply_received = True 

        elif self.step == PickCubeStep.RECUAR_CUBO and self.command_finished():
            time.sleep(0.25)
            self.step = PickCubeStep.GIRAR_VERTICAL2
            self.send_feedback(0.70, 'girando e movendo altura simultaneamente')
            self.send_command(f'COMBO:{self.slot_to_cmd(args[3])}:alt_giro')

        elif self.step == PickCubeStep.GIRAR_VERTICAL2 and self.command_finished():
            time.sleep(0.25)
            self.step = PickCubeStep.VERTICAL_CHASSI2
            self.send_feedback(0.80, 'abaixando para alturas dos slots')
            self.send_command('inicial')

        elif self.step == PickCubeStep.VERTICAL_CHASSI2 and self.command_finished():
            time.sleep(0.25)
            self.step = PickCubeStep.ABRIR
            self.send_feedback(0.90, 'abrindo garra')
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