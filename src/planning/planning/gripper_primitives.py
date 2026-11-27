from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from std_msgs.msg import String
from rclpy.duration import Duration
import os
import yaml
from ament_index_python.packages import get_package_share_directory
import math #mudei
from geometry_msgs.msg import Twist

class GripperPrimitives(ActionExecutorClient):
    def __init__(self, action_name: str):
        super().__init__(action_name, 0.25)

        self._pub = self.create_publisher(String, '/topico_garra', 10) #cria publisher para enviar ações a serem feitas
        self._sub = self.create_subscription(
            String, '/topico_garra_status', self._status_callback, 10) #cria subscriber de forma a receber o status de conclusão das ações
        
        self._pending_reply = None #qual reply de status é esperada de ser a próxima para a execução
        self._current_reply = None #ultima reply recebida
        self._reply_received = False #reply recebida, deve ser checada depois se é a esperada

        # Dicionário interno para armazenar os locais mapeados
        self.locations_dict = self._load_locations_from_params()
        self.get_logger().info(f'locations carregadas: {self.locations_dict}')

        self._pub_vel = self.create_publisher(Twist, '/cmd_vel', 10) #mudei
        self._vel_ativa = False
        self._vel_twist = None
        self._vel_fim = None
        self._timer_vel = self.create_timer(0.05, self._timer_vel_callback)  # 20 Hz

    def _status_callback(self, msg): #roda sempre que chega uma mensagem em /topico_garra_status
        self._current_reply = msg.data
        self._reply_received = True

    def send_command(self, cmd: str): #ação que é rodada sempre que é executada do_work() nas subclasses
        msg = String()
        msg.data = cmd
        self._pub.publish(msg) #monta e publica string para /topico_garra
        self._pending_reply = cmd #diz qual comando está esperando devolta no status
        self._reply_received = False #reseta reply_received para saber que espera-se um novo comando ser executado

    def command_finished(self) -> bool:
        return self._reply_received and self._current_reply == self._pending_reply #só retorna true se espera um reply por reply_received, e se é igual ao esperado

    def slot_to_cmd(self, slot: str) -> str:
        chave = String()

        mapping = {
            'slot1': 'slot1', 
            'slot2': 'slot2', 
            'slot3': 'slot3', 
            'pf': 'frente',
            'p1': 'pos1',
            'p2': 'pos2',
            'p3': 'pos3',
            'p4': 'pos4',
            # deve casar com comandos possíveis recebidos na main
        }

        if slot.startswith('p'): #posições devem ser padronizadas por p(1-3)l#
            chave = slot[:2]
            return mapping[chave]

        return mapping[slot]

    def height_to_cmd(self, location: str) -> str:
        mapping = {
            'loc1': '5',
            'loc2': '10',
            'loc3': '15',
            'loc4': '15', #bloqueio
            'loc5': '10',
            'loc6': '10',
            'loc7': '5',
            'loc8': '10',
            'loc9': '15',
            'loc10': '15', #bloqueio
            'locshelf1' : '5',
            'locshelf2' : 'shelf',
            'locshelf3' : '5',
            'locshelf4' : 'shelf',
            'locshelf5' : '5',
            'locshelf6' : 'shelf',
            'locshelf7' : '5',
            'locshelf8' : 'shelf',
            'locpp1' :'10',
            'locpp2' :'10'
            # colocar todas as locations e a altura de mesa correspondente
        }

        return mapping[location]

    # def _load_locations_from_params(self) -> dict:
    #     """Lê os parâmetros 'locations' do ROS 2 e converte para um dicionário python"""
    #     locations = {}
        
    #     # Pega todos os parâmetros carregados neste nó
    #     # No caso do seu YAML, eles ficarão salvos como 'locations.loc2', 'locations.loc3'
    #     params = self.get_parameters_by_prefix('locations')
        
    #     for key, param in params.items():
    #         # key será apenas o nome da localização (ex: 'loc2', 'start')
    #         # param.value conterá a lista de 4 posições (x, y, z, yaw)
    #         locations[key] = param.value
            
    #    return locations
    
    def _load_locations_from_params(self) -> dict:
        locations = {}
        self.declare_parameter('locations_file', '')
        path = self.get_parameter('locations_file').get_parameter_value().string_value

        # se o launch não passar o parâmetro, usa o arquivo do pacote
        if not path:
            path = os.path.join(get_package_share_directory('planning'),
                                'config', 'locations.yaml')

        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)

            for name, value in data['/**']['ros__parameters']['locations'].items():
                if isinstance(value, (list, tuple)) and len(value) == 3:
                    locations[name] = tuple(float(v) for v in value)

            self.get_logger().info(f'Localizações carregadas: {list(locations.keys())}')
        except Exception as e:
            self.get_logger().error(f'Erro ao carregar locations.yaml ({path}): {e}')

        return locations

    def get_location_angle(self, loc_name: str) -> float:
        if loc_name in self.locations_dict:
            # Pega o 4º valor do array [x, y, z, angulo]
            return float(self.locations_dict[loc_name][2])
        else:
            self.get_logger().error(f'Location {loc_name} nao encontrada no YAML de parametros!')
            return 0.0 # Valor default em caso de erro

    def start_translation(self, distancia_m: float, velocidade: float = 0.1):
        """Translada de lado em malha aberta. distancia_m > 0 = esquerda, < 0 = direita
        (mesma convenção do vision_node: +linear.y = esquerda)."""
        if distancia_m == 0.0:
            self._vel_ativa = False
            return
        duracao = abs(distancia_m) / velocidade
        self._vel_twist = Twist()
        self._vel_twist.linear.y = math.copysign(velocidade, distancia_m)
        self._vel_fim = self.get_clock().now() + Duration(seconds=duracao)
        self._vel_ativa = True

    def translation_finished(self) -> bool:
        return not self._vel_ativa

    def _timer_vel_callback(self):
        if not self._vel_ativa:
            return
        if self.get_clock().now() >= self._vel_fim:
            self._vel_ativa = False
            for _ in range(3):                 # garante que as rodas param
                self._pub_vel.publish(Twist())
            return
        self._pub_vel.publish(self._vel_twist)
        