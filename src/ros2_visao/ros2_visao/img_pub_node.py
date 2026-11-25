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

        # Inicializa a câmera com segurança no início (fica aberta em segundo plano, mas em silêncio)
        self.cameraDevice = 0 
        self.camera = cv2.VideoCapture(self.cameraDevice) 

        if not self.camera.isOpened():
            self.get_logger().error("ERRO CRÍTICO: Não foi possível conectar à câmera.")
        else:
            self.get_logger().info("Câmera inicializada com sucesso! Aguardando comando da garra...")
            for _ in range(30):
                self.camera.read()

        # Timer rodando em segundo plano, respeitando a flag 'self.publicando'
        self.periodCommunication = 0.033
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        self.i = 0

    def garra_callback(self, msg):
        cmd = msg.data
        if not cmd.startswith('transladar:'):
            return  # Ignora outros comandos

        self.get_logger().info(f'Comando recebido: {cmd} -> LIGANDO envio de imagens!')
        self.publicando = True  # Libera o timer para publicar

    def status_callback(self, msg):
        # NOVO: Quando o listener publica o eco em /topico_garra_status, a tarefa acabou!
        self.get_logger().info(f'Alinhamento concluído ({msg.data}) -> DESLIGANDO envio de imagens.')
        self.publicando = False  # Trava o timer novamente, parando de publicar

    def timer_callbackFunction(self):
        # Se a chave for falsa, não faz nada (economia total de recursos)
        if not self.publicando:
            return

        sucess, frame = self.camera.read()
        
        if sucess:
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
            ROS2ImageMessage = self.bridgeObject.cv2_to_imgmsg(frame, encoding='bgr8')

            self.publisher.publish(ROS2ImageMessage)
            self.i += 1
        else:
            self.get_logger().warn('Aguardando imagem da câmera...')

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
