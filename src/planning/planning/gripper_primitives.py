from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from std_msgs.msg import String
from rclpy.duration import Duration

class GripperPrimitives(ActionExecutorClient):
    def __init__(self, action_name: str):
        super().__init__(action_name, 0.25)

        # DECLARANDO E LENDO PARÂMETROS
        # Avisa ao ROS2 que queremos aceitar qualquer parâmetro no grupo "locations"

#        self.declare_parameters(
#            namespace='',
#            parameters=[
#                ('locations', None)
#            ]
#        )

        self._pub = self.create_publisher(String, '/topico_garra', 10) #cria publisher para enviar ações a serem feitas
        self._sub = self.create_subscription(
            String, '/topico_garra_status', self._status_callback, 10) #cria subscriber de forma a receber o status de conclusão das ações
        
        self._pending_reply = None #qual reply de status é esperada de ser a próxima para a execução
        self._current_reply = None #ultima reply recebida
        self._reply_received = False #reply recebida, deve ser checada depois se é a esperada

        # Dicionário interno para armazenar os locais mapeados
        self.locations_dict = self._load_locations_from_params()

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
            'p1': 'pos1',
            'p2': 'frente',
            'p3': 'pos3',
            # deve casar com comandos possíveis recebidos na main
        }

        if slot.startswith('p'): #posições devem ser padronizadas por p(1-3)l#
            chave = slot[:2]
            return mapping[chave]

        return mapping[slot]

    def height_to_cmd(self, location: str) -> str:
        mapping = {
            'loc1': '5cm',
            'loc2': '5cm',
            'loc3': '5cm',
            'loc4': '10cm',
            'loc5': '10cm',
            'loc6': '10cm',
            'loc7': '15cm',
            'loc8': '15cm',
            'loc9': '15cm',
            # colocar todas as locations e a altura de mesa correspondente
        }

        return mapping[location]

    def _load_locations_from_params(self) -> dict:
        """Lê os parâmetros 'locations' do ROS 2 e converte para um dicionário python"""
        locations = {}
        
        # Pega todos os parâmetros carregados neste nó
        # No caso do seu YAML, eles ficarão salvos como 'locations.loc2', 'locations.loc3'
        params = self.get_parameters_by_prefix('locations')
        
        for key, param in params.items():
            # key será apenas o nome da localização (ex: 'loc2', 'start')
            # param.value conterá a lista de 4 posições (x, y, z, yaw)
            locations[key] = param.value
            
        return locations

    def get_location_angle(self, loc_name: str) -> float:
        if loc_name in self.locations_dict:
            # Pega o 4º valor do array [x, y, z, angulo]
            return float(self.locations_dict[loc_name][3])
        else:
            self.get_logger().error(f'Location {loc_name} nao encontrada no YAML de parametros!')
            return 0.0 # Valor default em caso de erro
        