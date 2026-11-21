import rclpy
from .gripper_primitives import GripperPrimitives
from enum import Enum, auto
import time

class CavityStep(Enum):
    #quando os ultrassonicos detectarem um cobo, meio que tira uma "foto", como a torre e a camera possuem uma distância fixa
    #o cubo detectado sempre estará na mesma posição da camera/shot, logo deve ser analisado aquele apriltag específico para inspencionar
    #se o cubo deve ser pego, ou não, e continuar o sacneamento para próximos cubos
    START = auto()

    #Pegar CUBE1
    PICK_AJUSTAR1 = auto()
    PICK_TRANSLADAR1 = auto()
    PICK_VERTICAL_GIRO_IDA1 = auto()
    PICK_GIRAR_MESA1 = auto() 
    PICK_VERTICAL_MESA1 = auto() 
    PICK_FECHAR1 = auto()
    PICK_VERTICAL_GIRO_VOLTA1 = auto()
    PICK_GIRAR_CHASSI1 = auto()
    PICK_VERTICAL_SLOT1 = auto()
    PICK_ABRIR1 = auto()
    #Pegar CUBE2
    PICK_AJUSTAR2 = auto()
    PICK_TRANSLADAR2 = auto()
    PICK_VERTICAL_GIRO_IDA2 = auto()
    PICK_GIRAR_MESA2 = auto() 
    PICK_VERTICAL_MESA2 = auto() 
    PICK_FECHAR2 = auto()
    PICK_VERTICAL_GIRO_VOLTA2 = auto()
    PICK_GIRAR_CHASSI2 = auto()
    PICK_VERTICAL_SLOT2 = auto()
    PICK_ABRIR2 = auto()
    #Deixar CUBE1
    DEIXAR_VERTICAL_GIRO_SLOT1 = auto()
    DEIXAR_GIRAR_SLOT1 = auto() 
    DEIXAR_VERTICAL_SLOT1 = auto() 
    DEIXAR_FECHAR1 = auto()
    DEIXAR_VERTICAL_GIRO_IDA1 = auto()
    DEIXAR_GIRAR_MESA1 = auto()
    DEIXAR_VERTICAL_MESA1 = auto() 
    DEIXAR_ABRIR1 = auto()
    DEIXAR_VERTICAL_GIRO_VOLTA1 = auto() #ver de subir para uma altura pequena
    #DEIXAR_GIRAR_CHASSI1 = auto() #caso entre na frente da câmera
    #Pegar CUBE3
    PICK_AJUSTAR3 = auto()
    PICK_TRANSLADAR3 = auto()
    PICK_VERTICAL_MESA3 = auto() 
    PICK_FECHAR3 = auto()
    PICK_VERTICAL_GIRO_VOLTA3 = auto()
    PICK_GIRAR_CHASSI3 = auto()
    PICK_VERTICAL_SLOT3 = auto()
    PICK_ABRIR3 = auto()
    #Deixar CUBE2
    DEIXAR_VERTICAL_GIRO_SLOT2 = auto()
    DEIXAR_GIRAR_SLOT2 = auto() 
    DEIXAR_VERTICAL_SLOT2 = auto() 
    DEIXAR_FECHAR2 = auto()
    DEIXAR_VERTICAL_GIRO_IDA2 = auto()
    DEIXAR_GIRAR_MESA2 = auto()
    DEIXAR_VERTICAL_MESA2 = auto() 
    DEIXAR_ABRIR2 = auto()
    DEIXAR_VERTICAL_GIRO_VOLTA2 = auto()
    #Pegar CUBE4
    PICK_AJUSTAR4 = auto()
    PICK_TRANSLADAR4 = auto()
    PICK_VERTICAL_MESA4 = auto() 
    PICK_FECHAR4 = auto()
    PICK_VERTICAL_GIRO_VOLTA4 = auto()
    PICK_GIRAR_CHASSI4 = auto()
    PICK_VERTICAL_SLOT4 = auto()
    PICK_ABRIR4 = auto()
    #Deixar CUBE3
    DEIXAR_VERTICAL_GIRO_SLOT3 = auto()
    DEIXAR_GIRAR_SLOT3 = auto() 
    DEIXAR_VERTICAL_SLOT3 = auto() 
    DEIXAR_FECHAR3 = auto()
    DEIXAR_VERTICAL_GIRO_IDA3 = auto()
    DEIXAR_GIRAR_MESA3 = auto()
    DEIXAR_VERTICAL_MESA3 = auto() 
    DEIXAR_ABRIR3 = auto()
    DEIXAR_VERTICAL_GIRO_VOLTA3 = auto()
    #Pegar CUBE5
    PICK_AJUSTAR5 = auto()
    PICK_TRANSLADAR5 = auto()
    PICK_VERTICAL_MESA5 = auto() 
    PICK_FECHAR5 = auto()
    PICK_VERTICAL_GIRO_VOLTA5 = auto()
    PICK_GIRAR_CHASSI5 = auto()
    PICK_VERTICAL_SLOT5 = auto()
    PICK_ABRIR5 = auto()
    #Deixar CUBE4
    DEIXAR_VERTICAL_GIRO_SLOT4 = auto()
    DEIXAR_GIRAR_SLOT4 = auto() 
    DEIXAR_VERTICAL_SLOT4 = auto() 
    DEIXAR_FECHAR4 = auto()
    DEIXAR_VERTICAL_GIRO_IDA4 = auto()
    DEIXAR_GIRAR_MESA4 = auto()
    DEIXAR_VERTICAL_MESA4 = auto() 
    DEIXAR_ABRIR4 = auto()
    DEIXAR_VERTICAL_GIRO_VOLTA4 = auto()
    #Pegar CUBE6
    PICK_AJUSTAR6 = auto()
    PICK_TRANSLADAR6 = auto()
    PICK_VERTICAL_MESA6 = auto() 
    PICK_FECHAR6 = auto()
    PICK_VERTICAL_GIRO_VOLTA6 = auto()
    PICK_GIRAR_CHASSI6 = auto()
    PICK_VERTICAL_SLOT6 = auto()
    PICK_ABRIR6 = auto()
    #Deixar CUBE5
    DEIXAR_VERTICAL_GIRO_SLOT5 = auto()
    DEIXAR_GIRAR_SLOT5 = auto() 
    DEIXAR_VERTICAL_SLOT5 = auto() 
    DEIXAR_FECHAR5 = auto()
    DEIXAR_VERTICAL_GIRO_IDA5 = auto()
    DEIXAR_GIRAR_MESA5 = auto()
    DEIXAR_VERTICAL_MESA5 = auto() 
    DEIXAR_ABRIR5 = auto()
    DEIXAR_VERTICAL_GIRO_VOLTA5 = auto()
    #Deixar CUBE6
    DEIXAR_GIRAR_SLOT6 = auto()
    DEIXAR_VERTICAL_SLOT6 = auto()
    DEIXAR_FECHAR6 = auto()
    DEIXAR_AJUSTAR6 = auto()
    DEIXAR_TRANSLADAR6 = auto()
    DEIXAR_VERTICAL_GIRO_IDA6 = auto()
    DEIXAR_GIRAR_MESA6 = auto() 
    DEIXAR_VERTICAL_MESA6 = auto() 
    DEIXAR_ABRIR6 = auto()
    DEIXAR_VERTICAL_GIRO_VOLTA6 = auto()
    DEIXAR_GIRAR_CHASSI6 = auto()

#cubo1, 3, 5 -> slot1; cubo2, 4, 6 -> slot2
class CavityAction(GripperPrimitives):
    def __init__(self):
        super().__init__('pick_from_location') #nome precisa bater com nome da action no pddl domain
        self.step = CavityStep.START
        self.target_location = None
        self.angulo = None

    def do_work(self):
        args = self.current_arguments  # [robot, cube, location] guarda os parâmetros mandados pela ação

        #PEGAR CUBO N1 
        if self.step == CavityStep.START:
            self.step = CavityStep.PICK_AJUSTAR1
            self.target_location = args[2]
            self.angulo = self.get_location_angle(self.target_location)
            self.send_feedback(0.009, f'Ajustando distância de 20 cm e angulação de {self.angulo} graus da mesa')
            time.sleep(1)  # delay antes de publicar o comando
            self.send_command(f'ajustar:{self.angulo}') 
        
        elif self.step == CavityStep.PICK_AJUSTAR1 and self.command_finished():
            self.step = CavityStep.PICK_TRANSLADAR1
            self.send_feedback(0.018, f'Posicionando em frente ao {args[1]}')
            time.sleep(1)  # delay antes de publicar o comando
            self.send_command(f'transladar:{args[1]}:{self.height_to_cmd(args[2])}:{args[2]}') 

        elif self.step == CavityStep.PICK_TRANSLADAR1 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_GIRO_IDA1
            self.send_feedback(0.027, 'Levantando para altura de giro')
            self.send_command('alt_giro') #mudar comando para frente

        elif self.step == CavityStep.PICK_VERTICAL_GIRO_IDA1 and self.command_finished():
            self.step = CavityStep.PICK_GIRAR_MESA1
            self.send_feedback(0.036, 'Girando para pegar cubo')
            self.send_command('frente') #mudar comando para frente

        elif self.step == CavityStep.PICK_GIRAR_MESA1 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_MESA1
            self.send_feedback(0.045, f'Movendo para altura de {args[2]}')
            self.send_command(self.height_to_cmd(args[2]))

        elif self.step == CavityStep.PICK_VERTICAL_MESA1 and self.command_finished():
            self.step = CavityStep.PICK_FECHAR1
            self.send_feedback(0.054, 'Fechando garra')
            self.send_command('fechar') 

        elif self.step == CavityStep.PICK_FECHAR1 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_GIRO_VOLTA1
            self.send_feedback(0.063, f'Subindo com {args[1]} para altura de giro')
            self.send_command('alt_giro')

        elif self.step == CavityStep.PICK_VERTICAL_GIRO_VOLTA1 and self.command_finished():
            self.step = CavityStep.PICK_GIRAR_CHASSI1
            self.send_feedback(0.072, f'Girando para slot1') #revezar com slot2
            self.send_command('slot1')

        elif self.step == CavityStep.PICK_GIRAR_CHASSI1 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_SLOT1
            self.send_feedback(0.081, 'Abaixando para altura dos slots')
            self.send_command('inicial')

        elif self.step == CavityStep.PICK_VERTICAL_SLOT1 and self.command_finished():
            self.step = CavityStep.PICK_ABRIR1
            self.send_feedback(0.09, 'Abrindo garra')
            self.send_command('abrir')

        #PEGAR CUBO N2
        elif self.step == CavityStep.PICK_ABRIR1 and self.command_finished():
            self.step = CavityStep.PICK_AJUSTAR2
            self.target_location = args[2]
            self.angulo = self.get_location_angle(self.target_location)
            self.send_feedback(0.1, f'Ajustando distância de 20 cm e angulação de {self.angulo} graus da mesa')
            time.sleep(1)  # delay antes de publicar o comando
            self.send_command(f'ajustar:{self.angulo}') 
        
        elif self.step == CavityStep.PICK_AJUSTAR2 and self.command_finished():
            self.step = CavityStep.PICK_TRANSLADAR2
            self.send_feedback(0.109, f'Posicionando em frente ao cubo dois')
            time.sleep(1)  # delay antes de publicar o comando
            self.send_command(f'transladar:zona101:{self.height_to_cmd(args[2])}:{args[2]}') #sempre é publicado a posição da tag onde deve ser deixado o cubo anteriormente pego

        elif self.step == CavityStep.PICK_TRANSLADAR2 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_GIRO_IDA2
            self.send_feedback(0.118, 'Levantando para altura de giro')
            self.send_command('alt_giro') #mudar comando para frente

        elif self.step == CavityStep.PICK_VERTICAL_GIRO_IDA2 and self.command_finished():
            self.step = CavityStep.PICK_GIRAR_MESA2
            self.send_feedback(0.127, 'Girando para pegar cubo')
            self.send_command('frente') #mudar comando para frente

        elif self.step == CavityStep.PICK_GIRAR_MESA2 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_MESA2
            self.send_feedback(0.136, f'Movendo para altura de {args[2]}')
            self.send_command(self.height_to_cmd(args[2]))

        elif self.step == CavityStep.PICK_VERTICAL_MESA2 and self.command_finished():
            self.step = CavityStep.PICK_FECHAR2
            self.send_feedback(0.145, 'Fechando garra')
            self.send_command('fechar') 

        elif self.step == CavityStep.PICK_FECHAR2 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_GIRO_VOLTA2
            self.send_feedback(0.154, f'Subindo com cubo 2 para altura de giro')
            self.send_command('alt_giro')

        elif self.step == CavityStep.PICK_VERTICAL_GIRO_VOLTA2 and self.command_finished():
            self.step = CavityStep.PICK_GIRAR_CHASSI2
            self.send_feedback(0.163, f'Girando para slot2') #revezar com slot2
            self.send_command('slot2')

        elif self.step == CavityStep.PICK_GIRAR_CHASSI2 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_SLOT2
            self.send_feedback(0.172, 'Abaixando para altura dos slots')
            self.send_command('inicial')

        elif self.step == CavityStep.PICK_VERTICAL_SLOT2 and self.command_finished():
            self.step = CavityStep.PICK_ABRIR2
            self.send_feedback(0.181, 'Abrindo garra')
            self.send_command('abrir')


        #DEIXAR CUBO N1
        elif self.step == CavityStep.PICK_ABRIR2 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_SLOT1
            self.send_feedback(0.19, 'Levantando para altura de giro')
            self.send_command('alt_giro') #mudar comando para frente

        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_SLOT1 and self.command_finished():
            self.step = CavityStep.DEIXAR_GIRAR_SLOT1
            self.send_feedback(0.2, 'Girando para pegar slot1')
            self.send_command('slot1') #mudar comando para frente

        elif self.step == CavityStep.DEIXAR_GIRAR_SLOT1 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_SLOT1
            self.send_feedback(0.209, f'Movendo para altura dos slots')
            self.send_command('inicial')

        elif self.step == CavityStep.DEIXAR_VERTICAL_SLOT1 and self.command_finished():
            self.step = CavityStep.DEIXAR_FECHAR1
            self.send_feedback(0.218, 'Fechando garra')
            self.send_command('fechar') 

        elif self.step == CavityStep.DEIXAR_FECHAR1 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_IDA1
            self.send_feedback(0.227, f'Subindo com cubo 1 para altura de giro')
            self.send_command('alt_giro')

        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_IDA1 and self.command_finished():
            self.step = CavityStep.DEIXAR_GIRAR_MESA1
            self.send_feedback(0.236, f'Girando para mesa') #revezar com slot2
            self.send_command('frente')

        elif self.step == CavityStep.DEIXAR_GIRAR_MESA1 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_MESA1
            self.send_feedback(0.245, f'Movendo para altura de {args[2]}')
            self.send_command(self.height_to_cmd(args[2]))

        elif self.step == CavityStep.DEIXAR_VERTICAL_MESA1 and self.command_finished():
            self.step = CavityStep.DEIXAR_ABRIR1
            self.send_feedback(0.254, 'Abrindo garra')
            self.send_command('abrir')

        elif self.step == CavityStep.DEIXAR_ABRIR1 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_VOLTA1
            self.send_feedback(0.263, 'Subindo com garra para possibilitar ajuste para próximo cubo')
            self.send_command('inicial')
        
        #PEGAR CUBO N3
        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_VOLTA1 and self.command_finished():
            self.step = CavityStep.PICK_AJUSTAR3
            self.target_location = args[2]
            self.angulo = self.get_location_angle(self.target_location)
            self.send_feedback(0.272, f'Ajustando distância de 20 cm e angulação de {self.angulo} graus da mesa')
            time.sleep(1)  # delay antes de publicar o comando
            self.send_command(f'ajustar:{self.angulo}') 
        
        elif self.step == CavityStep.PICK_AJUSTAR3 and self.command_finished():
            self.step = CavityStep.PICK_TRANSLADAR3
            self.send_feedback(0.281, f'Posicionando em frente ao cubo três')
            time.sleep(1)  # delay antes de publicar o comando
            self.send_command(f'transladar:zona2:{self.height_to_cmd(args[2])}:{args[2]}') 

        elif self.step == CavityStep.PICK_TRANSLADAR3 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_MESA3
            self.send_feedback(0.29, f'Movendo para altura de {args[2]}')
            self.send_command(self.height_to_cmd(args[2]))

        elif self.step == CavityStep.PICK_VERTICAL_MESA3 and self.command_finished():
            self.step = CavityStep.PICK_FECHAR3
            self.send_feedback(0.3, 'Fechando garra')
            self.send_command('fechar') 

        elif self.step == CavityStep.PICK_FECHAR3 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_GIRO_VOLTA3
            self.send_feedback(0.309, f'Subindo com cubo 3 para altura de giro')
            self.send_command('alt_giro')

        elif self.step == CavityStep.PICK_VERTICAL_GIRO_VOLTA3 and self.command_finished():
            self.step = CavityStep.PICK_GIRAR_CHASSI3
            self.send_feedback(0.318, f'Girando para slot1') #revezar com slot2
            self.send_command('slot1')

        elif self.step == CavityStep.PICK_GIRAR_CHASSI3 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_SLOT3
            self.send_feedback(0.327, 'Abaixando para altura dos slots')
            self.send_command('inicial')

        elif self.step == CavityStep.PICK_VERTICAL_SLOT3 and self.command_finished():
            self.step = CavityStep.PICK_ABRIR3
            self.send_feedback(0.336, 'Abrindo garra')
            self.send_command('abrir')

        #DEIXAR CUBO N2
        elif self.step == CavityStep.PICK_ABRIR3 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_SLOT2
            self.send_feedback(0.345, 'Levantando para altura de giro')
            self.send_command('alt_giro') #mudar comando para frente

        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_SLOT2 and self.command_finished():
            self.step = CavityStep.DEIXAR_GIRAR_SLOT2
            self.send_feedback(0.354, 'Girando para pegar slot2')
            self.send_command('slot2') #mudar comando para frente

        elif self.step == CavityStep.DEIXAR_GIRAR_SLOT2 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_SLOT2
            self.send_feedback(0.363, f'Movendo para altura dos slots')
            self.send_command('inicial')

        elif self.step == CavityStep.DEIXAR_VERTICAL_SLOT2 and self.command_finished():
            self.step = CavityStep.DEIXAR_FECHAR2
            self.send_feedback(0.372, 'Fechando garra')
            self.send_command('fechar') 

        elif self.step == CavityStep.DEIXAR_FECHAR2 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_IDA2
            self.send_feedback(0.381, f'Subindo com cubo 2 para altura de giro')
            self.send_command('alt_giro')

        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_IDA2 and self.command_finished():
            self.step = CavityStep.DEIXAR_GIRAR_MESA2
            self.send_feedback(0.39, f'Girando para mesa') #revezar com slot2
            self.send_command('frente')

        elif self.step == CavityStep.DEIXAR_GIRAR_MESA2 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_MESA2
            self.send_feedback(0.4, f'Movendo para altura de {args[2]}')
            self.send_command(self.height_to_cmd(args[2]))

        elif self.step == CavityStep.DEIXAR_VERTICAL_MESA2 and self.command_finished():
            self.step = CavityStep.DEIXAR_ABRIR2
            self.send_feedback(0.409, 'Abrindo garra')
            self.send_command('abrir')

        elif self.step == CavityStep.DEIXAR_ABRIR2 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_VOLTA2
            self.send_feedback(0.418, 'Subindo com garra para possibilitar ajuste para próximo cubo')
            self.send_command('inicial')

        #PEGAR CUBO N4
        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_VOLTA2 and self.command_finished():
            self.step = CavityStep.PICK_AJUSTAR4
            self.target_location = args[2]
            self.angulo = self.get_location_angle(self.target_location)
            self.send_feedback(0.427, f'Ajustando distância de 20 cm e angulação de {self.angulo} graus da mesa')
            time.sleep(1)  # delay antes de publicar o comando
            self.send_command(f'ajustar:{self.angulo}') 
        
        elif self.step == CavityStep.PICK_AJUSTAR4 and self.command_finished():
            self.step = CavityStep.PICK_TRANSLADAR4
            self.send_feedback(0.436, f'Posicionando em frente ao cubo quatro')
            time.sleep(1)  # delay antes de publicar o comando
            self.send_command(f'transladar:zone3:{self.height_to_cmd(args[2])}:{args[2]}') 

        elif self.step == CavityStep.PICK_TRANSLADAR4 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_MESA4
            self.send_feedback(0.445, f'Movendo para altura de {args[2]}')
            self.send_command(self.height_to_cmd(args[2]))

        elif self.step == CavityStep.PICK_VERTICAL_MESA4 and self.command_finished():
            self.step = CavityStep.PICK_FECHAR4
            self.send_feedback(0.454, 'Fechando garra')
            self.send_command('fechar') 

        elif self.step == CavityStep.PICK_FECHAR4 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_GIRO_VOLTA4
            self.send_feedback(0.463, f'Subindo com cubo 4 para altura de giro')
            self.send_command('alt_giro')

        elif self.step == CavityStep.PICK_VERTICAL_GIRO_VOLTA4 and self.command_finished():
            self.step = CavityStep.PICK_GIRAR_CHASSI4
            self.send_feedback(0.472, f'Girando para slot2') #revezar com slot2
            self.send_command('slot2')

        elif self.step == CavityStep.PICK_GIRAR_CHASSI4 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_SLOT4
            self.send_feedback(0.481, 'Abaixando para altura dos slots')
            self.send_command('inicial')

        elif self.step == CavityStep.PICK_VERTICAL_SLOT4 and self.command_finished():
            self.step = CavityStep.PICK_ABRIR4
            self.send_feedback(0.49, 'Abrindo garra')
            self.send_command('abrir')

        #DEIXAR CUBO N3
        elif self.step == CavityStep.PICK_ABRIR4 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_SLOT3
            self.send_feedback(0.5, 'Levantando para altura de giro')
            self.send_command('alt_giro') #mudar comando para frente

        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_SLOT3 and self.command_finished():
            self.step = CavityStep.DEIXAR_GIRAR_SLOT3
            self.send_feedback(0.509, 'Girando para pegar slot1')
            self.send_command('slot1') #mudar comando para frente

        elif self.step == CavityStep.DEIXAR_GIRAR_SLOT3 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_SLOT3
            self.send_feedback(0.518, f'Movendo para altura dos slots')
            self.send_command('inicial')

        elif self.step == CavityStep.DEIXAR_VERTICAL_SLOT3 and self.command_finished():
            self.step = CavityStep.DEIXAR_FECHAR3
            self.send_feedback(0.527, 'Fechando garra')
            self.send_command('fechar') 

        elif self.step == CavityStep.DEIXAR_FECHAR3 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_IDA3
            self.send_feedback(0.536, f'Subindo com cubo 3 para altura de giro')
            self.send_command('alt_giro')

        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_IDA3 and self.command_finished():
            self.step = CavityStep.DEIXAR_GIRAR_MESA3
            self.send_feedback(0.545, f'Girando para mesa') #revezar com slot2
            self.send_command('frente')

        elif self.step == CavityStep.DEIXAR_GIRAR_MESA3 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_MESA3
            self.send_feedback(0.554, f'Movendo para altura de {args[2]}')
            self.send_command(self.height_to_cmd(args[2]))

        elif self.step == CavityStep.DEIXAR_VERTICAL_MESA3 and self.command_finished():
            self.step = CavityStep.DEIXAR_ABRIR3
            self.send_feedback(0.563, 'Abrindo garra')
            self.send_command('abrir')

        elif self.step == CavityStep.DEIXAR_ABRIR3 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_VOLTA3
            self.send_feedback(0.572, 'Subindo com garra para possibilitar ajuste para próximo cubo')
            self.send_command('inicial')

        #PEGAR CUBO N5
        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_VOLTA3 and self.command_finished():
            self.step = CavityStep.PICK_AJUSTAR5
            self.target_location = args[2]
            self.angulo = self.get_location_angle(self.target_location)
            self.send_feedback(0.581, f'Ajustando distância de 20 cm e angulação de {self.angulo} graus da mesa')
            time.sleep(1)  # delay antes de publicar o comando
            self.send_command(f'ajustar:{self.angulo}') 
        
        elif self.step == CavityStep.PICK_AJUSTAR5 and self.command_finished():
            self.step = CavityStep.PICK_TRANSLADAR5
            self.send_feedback(0.59, f'Posicionando em frente ao cubo cinco')
            time.sleep(1)  # delay antes de publicar o comando
            self.send_command(f'transladar:zone4:{self.height_to_cmd(args[2])}:{args[2]}') 

        elif self.step == CavityStep.PICK_TRANSLADAR5 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_MESA5
            self.send_feedback(0.6, f'Movendo para altura de {args[2]}')
            self.send_command(self.height_to_cmd(args[2]))

        elif self.step == CavityStep.PICK_VERTICAL_MESA5 and self.command_finished():
            self.step = CavityStep.PICK_FECHAR5
            self.send_feedback(0.609, 'Fechando garra')
            self.send_command('fechar') 

        elif self.step == CavityStep.PICK_FECHAR5 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_GIRO_VOLTA5
            self.send_feedback(0.618, f'Subindo com cubo 5 para altura de giro')
            self.send_command('alt_giro')

        elif self.step == CavityStep.PICK_VERTICAL_GIRO_VOLTA5 and self.command_finished():
            self.step = CavityStep.PICK_GIRAR_CHASSI5
            self.send_feedback(0.627, f'Girando para slot1') #revezar com slot2
            self.send_command('slot1')

        elif self.step == CavityStep.PICK_GIRAR_CHASSI5 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_SLOT5
            self.send_feedback(0.636, 'Abaixando para altura dos slots')
            self.send_command('inicial')

        elif self.step == CavityStep.PICK_VERTICAL_SLOT5 and self.command_finished():
            self.step = CavityStep.PICK_ABRIR5
            self.send_feedback(0.645, 'Abrindo garra')
            self.send_command('abrir')

        #DEIXAR CUBO N4
        elif self.step == CavityStep.PICK_ABRIR5 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_SLOT4
            self.send_feedback(0.654, 'Levantando para altura de giro')
            self.send_command('alt_giro') #mudar comando para frente

        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_SLOT4 and self.command_finished():
            self.step = CavityStep.DEIXAR_GIRAR_SLOT4
            self.send_feedback(0.663, 'Girando para pegar slot2')
            self.send_command('slot2') #mudar comando para frente

        elif self.step == CavityStep.DEIXAR_GIRAR_SLOT4 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_SLOT4
            self.send_feedback(0.672, f'Movendo para altura dos slots')
            self.send_command('inicial')

        elif self.step == CavityStep.DEIXAR_VERTICAL_SLOT4 and self.command_finished():
            self.step = CavityStep.DEIXAR_FECHAR4
            self.send_feedback(0.681, 'Fechando garra')
            self.send_command('fechar') 

        elif self.step == CavityStep.DEIXAR_FECHAR4 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_IDA4
            self.send_feedback(0.69, f'Subindo com cubo 4 para altura de giro')
            self.send_command('alt_giro')

        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_IDA4 and self.command_finished():
            self.step = CavityStep.DEIXAR_GIRAR_MESA4
            self.send_feedback(0.7, f'Girando para mesa') #revezar com slot2
            self.send_command('frente')

        elif self.step == CavityStep.DEIXAR_GIRAR_MESA4 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_MESA4
            self.send_feedback(0.709, f'Movendo para altura de {args[2]}')
            self.send_command(self.height_to_cmd(args[2]))

        elif self.step == CavityStep.DEIXAR_VERTICAL_MESA4 and self.command_finished():
            self.step = CavityStep.DEIXAR_ABRIR4
            self.send_feedback(0.718, 'Abrindo garra')
            self.send_command('abrir')

        elif self.step == CavityStep.DEIXAR_ABRIR4 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_VOLTA4
            self.send_feedback(0.727, 'Subindo com garra para possibilitar ajuste para próximo cubo')
            self.send_command('inicial')

        #PEGAR CUBO N6
        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_VOLTA4 and self.command_finished():
            self.step = CavityStep.PICK_AJUSTAR6
            self.target_location = args[2]
            self.angulo = self.get_location_angle(self.target_location)
            self.send_feedback(0.736, f'Ajustando distância de 20 cm e angulação de {self.angulo} graus da mesa')
            time.sleep(1)  # delay antes de publicar o comando
            self.send_command(f'ajustar:{self.angulo}') 
        
        elif self.step == CavityStep.PICK_AJUSTAR6 and self.command_finished():
            self.step = CavityStep.PICK_TRANSLADAR6
            self.send_feedback(0.745, f'Posicionando em frente ao cubo seis')
            time.sleep(1)  # delay antes de publicar o comando
            self.send_command(f'transladar:zone5:{self.height_to_cmd(args[2])}:{args[2]}') 

        elif self.step == CavityStep.PICK_TRANSLADAR6 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_MESA6
            self.send_feedback(0.754, f'Movendo para altura de {args[2]}')
            self.send_command(self.height_to_cmd(args[2]))

        elif self.step == CavityStep.PICK_VERTICAL_MESA6 and self.command_finished():
            self.step = CavityStep.PICK_FECHAR6
            self.send_feedback(0.763, 'Fechando garra')
            self.send_command('fechar') 

        elif self.step == CavityStep.PICK_FECHAR6 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_GIRO_VOLTA6
            self.send_feedback(0.772, f'Subindo com cubo 6 para altura de giro')
            self.send_command('alt_giro')

        elif self.step == CavityStep.PICK_VERTICAL_GIRO_VOLTA6 and self.command_finished():
            self.step = CavityStep.PICK_GIRAR_CHASSI6
            self.send_feedback(0.781, f'Girando para slot2') #revezar com slot2
            self.send_command('slot2')

        elif self.step == CavityStep.PICK_GIRAR_CHASSI6 and self.command_finished():
            self.step = CavityStep.PICK_VERTICAL_SLOT6
            self.send_feedback(0.79, 'Abaixando para altura dos slots')
            self.send_command('inicial')

        elif self.step == CavityStep.PICK_VERTICAL_SLOT6 and self.command_finished():
            self.step = CavityStep.PICK_ABRIR6
            self.send_feedback(0.8, 'Abrindo garra')
            self.send_command('abrir')

        #DEIXAR CUBO N5
        elif self.step == CavityStep.PICK_ABRIR6 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_SLOT5
            self.send_feedback(0.809, 'Levantando para altura de giro')
            self.send_command('alt_giro') #mudar comando para frente

        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_SLOT5 and self.command_finished():
            self.step = CavityStep.DEIXAR_GIRAR_SLOT5
            self.send_feedback(0.818, 'Girando para pegar slot1')
            self.send_command('slot1') #mudar comando para frente

        elif self.step == CavityStep.DEIXAR_GIRAR_SLOT5 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_SLOT5
            self.send_feedback(0.827, f'Movendo para altura dos slots')
            self.send_command('inicial')

        elif self.step == CavityStep.DEIXAR_VERTICAL_SLOT5 and self.command_finished():
            self.step = CavityStep.DEIXAR_FECHAR5
            self.send_feedback(0.836, 'Fechando garra')
            self.send_command('fechar') 

        elif self.step == CavityStep.DEIXAR_FECHAR5 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_IDA5
            self.send_feedback(0.845, f'Subindo com cubo 5 para altura de giro')
            self.send_command('alt_giro')

        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_IDA5 and self.command_finished():
            self.step = CavityStep.DEIXAR_GIRAR_MESA5
            self.send_feedback(0.854, f'Girando para mesa') #revezar com slot2
            self.send_command('frente')

        elif self.step == CavityStep.DEIXAR_GIRAR_MESA5 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_MESA5
            self.send_feedback(0.863, f'Movendo para altura de {args[2]}')
            self.send_command(self.height_to_cmd(args[2]))

        elif self.step == CavityStep.DEIXAR_VERTICAL_MESA5 and self.command_finished():
            self.step = CavityStep.DEIXAR_ABRIR5
            self.send_feedback(0.872, 'Abrindo garra')
            self.send_command('abrir')

        elif self.step == CavityStep.DEIXAR_ABRIR5 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_VOLTA5
            self.send_feedback(0.881, 'Subindo com garra para altura de giro para pegar último cubo')
            self.send_command('alt_giro')

        #DEIXAR CUBO N6
        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_VOLTA5 and self.command_finished():
            self.step = CavityStep.DEIXAR_GIRAR_SLOT6
            self.send_feedback(0.89, 'Girando para pegar slot2')
            self.send_command('slot2') #mudar comando para frente

        elif self.step == CavityStep.DEIXAR_GIRAR_SLOT6 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_SLOT6
            self.send_feedback(0.9, f'Movendo para altura dos slots')
            self.send_command('inicial')

        elif self.step == CavityStep.DEIXAR_VERTICAL_SLOT6 and self.command_finished():
            self.step = CavityStep.DEIXAR_FECHAR6
            self.send_feedback(0.909, 'Fechando garra')
            self.send_command('fechar')

        elif self.step == CavityStep.DEIXAR_FECHAR6 and self.command_finished():
            self.step = CavityStep.DEIXAR_AJUSTAR6
            self.target_location = args[2]
            self.angulo = self.get_location_angle(self.target_location)
            self.send_feedback(0.918, f'Ajustando distância de 20 cm e angulação de {self.angulo} graus da mesa')
            time.sleep(1)  # delay antes de publicar o comando
            self.send_command(f'ajustar:{self.angulo}') 
        
        elif self.step == CavityStep.DEIXAR_AJUSTAR6 and self.command_finished():
            self.step = CavityStep.DEIXAR_TRANSLADAR6
            self.send_feedback(0.927, f'Posicionando em frente ao slot do cubo 6')
            time.sleep(1)  # delay antes de publicar o comando
            self.send_command(f'transladar:zone6:{self.height_to_cmd(args[2])}:{args[2]}')  

        elif self.step == CavityStep.DEIXAR_TRANSLADAR6 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_IDA6
            self.send_feedback(0.936, f'Subindo com cubo 6 para altura de giro')
            self.send_command('alt_giro')

        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_IDA6 and self.command_finished():
            self.step = CavityStep.DEIXAR_GIRAR_MESA6
            self.send_feedback(0.945, f'Girando para mesa') #revezar com slot2
            self.send_command('frente')

        elif self.step == CavityStep.DEIXAR_GIRAR_MESA6 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_MESA6
            self.send_feedback(0.954, f'Movendo para altura de {args[2]}')
            self.send_command(self.height_to_cmd(args[2]))

        elif self.step == CavityStep.DEIXAR_VERTICAL_MESA6 and self.command_finished():
            self.step = CavityStep.DEIXAR_ABRIR6
            self.send_feedback(0.963, 'Abrindo garra')
            self.send_command('abrir')

        elif self.step == CavityStep.DEIXAR_ABRIR6 and self.command_finished():
            self.step = CavityStep.DEIXAR_VERTICAL_GIRO_VOLTA6
            self.send_feedback(0.972, 'Subindo com garra para altura de giro')
            self.send_command('alt_giro')

        elif self.step == CavityStep.DEIXAR_VERTICAL_GIRO_VOLTA6 and self.command_finished():
            self.step = CavityStep.DEIXAR_GIRAR_CHASSI6
            self.send_feedback(0.981, 'Girando garra para posição de repouso')
            self.send_command('slot2')

        elif self.step == CavityStep.DEIXAR_GIRAR_CHASSI6 and self.command_finished():
            self.finish(True, 1.0, 'place-at-cavity concluído')     


def main(args=None):
    rclpy.init(args=args)
    node = CavityAction()
    node.trigger_configure()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
