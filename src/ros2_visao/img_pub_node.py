import cv2 
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node 
from cv_bridge import CvBridge 

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('img_pub_node')
        
        self.cameraDevice = 2
        self.camera = cv2.VideoCapture(self.cameraDevice) 

        if not self.camera.isOpened():
           self.get_logger().error("ERRO CRÍTICO: Não foi possível conectar ao DroidCam. Verifique o IP ou o aplicativo no celular.")
        else:
           self.get_logger().info("Conectado ao DroidCam com sucesso!")
        for i in range(30):
           self.camera.read()

        
        
        self.bridgeObject = CvBridge()
        self.topicNameFrames = '/cam/image_raw'
        self.queueSize = 10 
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)

        self.publisher
        
        self.periodCommunication = 0.033
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        self.i = 0

    def timer_callbackFunction(self):
        sucess, frame = self.camera.read()
        
        if sucess:
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
            
            
            ROS2ImageMessage = self.bridgeObject.cv2_to_imgmsg(frame, encoding='bgr8')

            #ROS2ImageMessage.header.stamp = self.get_clock().now().to_msg() # Preenche o tempo atual
            #ROS2ImageMessage.header.frame_id = 'camera_frame'               # Identificador do sensor

            self.publisher.publish(ROS2ImageMessage)
            self.get_logger().info('Publishing image number %d' % self.i)
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
        publisherObject.camera.release()
        publisherObject.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()