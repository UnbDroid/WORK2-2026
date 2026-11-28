import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('img_pub_node')

        # 1. Escuta a garra para LIGAR
        self.sub_garra = self.create_subscription(String, '/topico_garra', self.garra_callback, 10)
        # 2. NOVO: Escuta o status do listener para DESLIGAR quando a tarefa acabar
        self.sub_status = self.create_subscription(String, '/topico_garra_status', self.status_callback, 10)

        # Configurações básicas do ROS 2
        self.bridgeObject = CvBridge()
        self.topicNameFrames = '/cam/image_raw'
        self.queueSize = 10
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)

        # Chave de controle (Começa falso)
        self.publicando = False
        self.cmd_ativo = None   # comando de visão que ligou a câmera (espera a resposta dele)
        self.t_ligou = None

        # Inicializa a câmera com segurança no início (fica aberta em segundo plano, mas em silêncio)
        # self.cameraDevice = 0
        # self.camera = cv2.VideoCapture(self.cameraDevice)
        #Eu mudei essas 2 aqui, o original é o comentado
        self.cameraDevice = "/dev/v4l/by-id/usb-046d_Logitech_BRIO_DA982BC6-video-index0"
        self.camera = cv2.VideoCapture(self.cameraDevice, cv2.CAP_V4L2)

        if not self.camera.isOpened():
            self.get_logger().error("(talker) ERRO CRÍTICO: Não foi possível conectar à câmera.")
        else:
            self.get_logger().info("(talker) Câmera inicializada com sucesso! Aguardando comando da garra...")
            for _ in range(30):
                self.camera.read()

        # Timer rodando em segundo plano, respeitando a flag 'self.publicando'
        self.periodCommunication = 0.033
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        self.i = 0

    # Comandos que precisam da câmera (todos atendidos pelo nó de visão, o listener)
    COMANDOS_VISAO = ('transladar:', 'inspecionar:', 'guardar')
    # Segurança: se a resposta nunca vier, a câmera desliga sozinha depois deste tempo (s)
    TEMPO_MAX_LIGADA = 300.0

    def garra_callback(self, msg):
        cmd = msg.data
        if not cmd.startswith(self.COMANDOS_VISAO):
            return  # Ignora outros comandos (garra, ajustar, impossivel...)

        self.get_logger().info(f'(talker) Comando recebido: {cmd} -> LIGANDO envio de imagens!')
        self.cmd_ativo = cmd
        self.t_ligou = self.get_clock().now()
        self.publicando = True  # Libera o timer para publicar

    def e_resposta_da_visao(self, status):
        """True se 'status' é a resposta do comando de visão que ligou a câmera.
        transladar e guardar: eco igual ao comando. inspecionar:zoneN: 'inspecionar:zoneN:<resultado>'."""
        cmd = getattr(self, 'cmd_ativo', None)
        if cmd is None:
            return False
        if cmd.startswith('inspecionar:'):
            return status.startswith(cmd + ':')
        if cmd == 'guardar':
            return status.startswith('guardar')
        return status == cmd

    def status_callback(self, msg):
        # Desliga só quando chega a resposta do comando de visão em andamento. Antes desligava
        # com QUALQUER status: o eco do transladar desligava a câmera antes do inspecionar
        # (que não religava) e o inspecionar nunca recebia imagem.
        if not self.publicando or not self.e_resposta_da_visao(msg.data):
            return
        self.get_logger().info(f'(talker) Visão concluiu ({msg.data}) -> DESLIGANDO envio de imagens.')
        self.publicando = False  # Trava o timer novamente, parando de publicar
        self.cmd_ativo = None

    def timer_callbackFunction(self):
        # Se a chave for falsa, não faz nada (economia total de recursos)
        if not self.publicando:
            return

        if (self.get_clock().now() - self.t_ligou).nanoseconds / 1e9 > self.TEMPO_MAX_LIGADA:
            self.get_logger().warn(
                f'(talker) Sem resposta da visão para {self.cmd_ativo} em {self.TEMPO_MAX_LIGADA:.0f} s -> DESLIGANDO.')
            self.publicando = False
            self.cmd_ativo = None
            return

        sucess, frame = self.camera.read()
        if sucess:
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
            ROS2ImageMessage = self.bridgeObject.cv2_to_imgmsg(frame, encoding='bgr8')

            self.publisher.publish(ROS2ImageMessage)
            self.i += 1
        else:
            self.get_logger().warn('(talker) Aguardando imagem da câmera...')

def main(args=None):
    rclpy.init(args=args)
    publisherObject = ImagePublisher()
    try:
        rclpy.spin(publisherObject)
    except KeyboardInterrupt:
        pass
    finally:
        if publisherObject.camera.isOpened():
            publisherObject.camera.release()
        publisherObject.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
 