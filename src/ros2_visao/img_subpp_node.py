import cv2 
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node 
from cv_bridge import CvBridge 
import numpy as np


def detectar_cores(frame):
    imgResultado = frame.copy() # cópia p/ desenhar os resultados finais
    
    # passo 2: blur para reduzir ruidos
    suave = cv2.GaussianBlur(frame, (7, 7), 0) # aplica blur
    (T, bin) = cv2.threshold(suave, 160, 255, cv2.THRESH_BINARY)
    
    # passo 3: conversão pra hsv
    img_hsv = cv2.cvtColor(suave, cv2.COLOR_BGR2HSV)
        
    #definição do intervalo de cores para detectar o vermelho (em HSV) 
    #primeira mascara (para o vermelho mais claro)
    red_lower = np.array([0,120,70])
    red_upper = np.array([10,255,255])
    red_mask1 = cv2.inRange(img_hsv, red_lower, red_upper)
    
    #segunda mascara (para o vermelho mais escuro)
    red_lower2 = np.array([170,120,70])
    red_upper2 = np.array([180,255,255])
    red_mask2 = cv2.inRange(img_hsv, red_lower2, red_upper2)
    
    # máscara final (só com vermelho pois usamos os dois tons de vermelho)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        
    #Dilatação e erosão para remover pequenos ruídos da máscara vermelha 
    kernel = np.ones((3,3), np.uint8)
    red_erode = cv2.erode(red_mask,kernel,iterations=1)
    red_dilate = cv2.dilate(red_erode,kernel,iterations=1)
        
    #Definição de intervalo de cores e máscara para tons de azul (em HSV)
    blue_lower = np.array([90, 50, 50])
    blue_upper = np.array([130, 255, 255])
    blue_mask = cv2.inRange(img_hsv, blue_lower, blue_upper)
    
        
    
    #Dilatação e erosão para remover pequenos ruídos da máscara azul
    kernel = np.ones((3,3), np.uint8)
    blue_erode = cv2.erode(blue_mask,kernel,iterations=1)
    blue_dilate = cv2.dilate(blue_erode,kernel,iterations=1) 

    #Processamento e Contagem de Contornos
    name = ["Vermelho", "Azul"]
    color = [(0, 0, 255), (255, 0, 0)] # BGR
    array = [red_dilate, blue_dilate]

    cont_x = 0
    contagem = {}

    for i in range(len(array)):
        # Busca contornos (Compatível com OpenCV 3+)
        contornos, _ = cv2.findContours(array[i], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        qtd = len(contornos)
        contagem[name[i]] = qtd

        # Desenha os contornos
        for x in range(qtd):
            cv2.drawContours(imgResultado, contornos, x, color[i], 2)

        # Escreve o texto da contagem na imagem
        cv2.putText(imgResultado, f"{name[i]}: {qtd}", (10 + cont_x, 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color[i], 1, cv2.LINE_AA)
        cont_x += 110

    return imgResultado 

    


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('img_subpp_node')
        self.brigeObject = CvBridge()
        self.topicNameFrames = '/cam/image_raw'
        self.subscription = self.create_subscription(
            Image, self.topicNameFrames, self.listener_callbackFunction, 10
        )
        
        # Força o tamanho correto da janela no servidor gráfico
        cv2.namedWindow("Detecção de cores", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Detecção de cores", 640, 480)

    def listener_callbackFunction(self, img):
        # LOG REMOVIDO DAQUI para não travar o teclado com texto infinito
        
        try:
            frame = self.brigeObject.imgmsg_to_cv2(img, desired_encoding='bgr8')
            vamover = detectar_cores(frame) 
            
            cv2.imshow("Detecção de cores", vamover)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Erro na conversão do frame: {e}')
        
def main (args=None):
        rclpy.init(args=args)
        image_subscriber = ImageSubscriber()
        rclpy.spin(image_subscriber)
        image_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()
