
class ImagePublisher(Node):
    def __init__(self):
        super().__init__('img_pub_node')

        # 1. Inscrição para escutar a garra
        self.sub_garra = self.create_subscription(String, '/topico_garra', self.garra_callback, 10)

        # 2. Configurações do ROS 2
        self.bridgeObject = CvBridge()
        self.topicNameFrames = '/cam/image_raw'
        self.queueSize = 10 
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)

        # 3. Chave de controle (Começa falso para não publicar nada)
        self.publicando = False

        # 4. Inicializa a câmera com segurança logo na subida do nó
        self.cameraDevice = 0 
        self.camera = cv2.VideoCapture(self.cameraDevice) 

        if not self.camera.isOpened():
            self.get_logger().error("ERRO CRÍTICO: Não foi possível conectar à câmera.")
        else:
            self.get_logger().info("Câmera inicializada com sucesso! Aguardando comando da garra...")
            for _ in range(30):
                self.camera.read()

        # 5. Timer criado, mas que respeita a chave 'self.publicando'
        self.periodCommunication = 0.033
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        self.i = 0

    def garra_callback(self, msg):
        cmd = msg.data
        if not cmd.startswith('transladar:'):
            return  # Ignora outros comandos (fechar, girar, etc.)
          
        self.get_logger().info(f'Comando recebido: {cmd} -> Liberando envio de imagens!')
        self.publicando = True  # Libera a publicação no timer

    def timer_callbackFunction(self):
        # Se a garra não mandou o comando ainda, pula o ciclo
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
