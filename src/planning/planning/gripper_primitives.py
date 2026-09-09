from plansys2_support_py.action_executor_client import ActionExecutorClient
from std_msgs.msg import String
from rclpy.duration import Duration

class GripperPrimitives(ActionExecutorClient):
    def __init__(self, action_name: str):
        super().__init__(action_name, 0.25)
        self._pub = self.create_publisher(String, '/topico_garra', 10) #cria publisher para enviar ações a serem feitas
        self._sub = self.create_subscription(
            String, '/topico_garra_status', self._status_callback, 10) #cria subscriber de forma a receber o status de conclusão das ações
        self._pending_reply = None #qual reply de status é esperada de ser a próxima para a execução
        self._current_reply = None #ultima reply recebida
        self._reply_received = False #reply recebida, deve ser checada depois se é a esperada

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
        # mapping = {'slot1': '1', 'slot2': '2', 'slot3': '3', 'origin': '0'}, só é necessário se comandos enviados pelo PDDL não batem exatamente com aqueles esperado pelo nó subscriber
        return slot #mapping[slot]

    def height_to_cmd(self, height: str) -> str:
        return height  # se já vier como "5cm"/"10cm"/"15cm"/"shelf"