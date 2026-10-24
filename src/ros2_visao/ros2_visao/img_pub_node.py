import cv2 
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node 
from cv_bridge import CvBridge 
from std_msgs.msg import String

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('img_pub_node')

        # 1. INSCRIÇÃO ESSENCIAL: Faltava escutar o tópico da garra aqui!
        self.sub_garra = self.create_subscription(String, '/topico_garra', self.garra_callback, 10)

        # 2. Configurações básicas
        self.bridgeObject = CvBridge()
        self.topicNameFrames = '/cam/image_raw'
        self.queueSize = 10 
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)

        # 3. Variável de controle (Começa travado, só publica após o comando)
        self.publicando = False

        # 4. Inicializa a câmera com segurança no início
        self.cameraDevice = 0
        self.camera = cv2.VideoCapture(self.cameraDevice) 

        if not self.camera.isOpened():
            self.get_logger().error("ERRO CRÍTICO: Não foi possível conectar à câmera.")
        else:
            self.get_logger().info("Câmera inicializada com sucesso! Aguardando comando da garra...")
            for _ in range(30):
                self.camera.read()

        # 5. Timer rodando em segundo plano, mas controlado pela flag 'self.publicando'
        self.periodCommunication = 0.033
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        self.i = 0

    def garra_callback(self, msg):
        cmd = msg.data
        if not cmd.startswith('transladar:'):
            return  # Ignora outros comandos da garra (fechar, girar, etc.)

        self.get_logger().info(f'Comando recebido: {cmd} -> Liberando envio de imagens!')
        self.publicando = True  # Libera o timer para comecar a publicar

    def timer_callbackFunction(self):
        # Se a garra ainda não mandou o comando de transladar, não faz nada
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
        # Fecha a câmera com segurança apenas se ela foi aberta
        if publisherObject.camera.isOpened():
            publisherObject.camera.release()
        publisherObject.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
