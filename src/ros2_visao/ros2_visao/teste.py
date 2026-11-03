from enum import Enum, auto
import re
import cv2
import numpy as np
import apriltag
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# 1. CLASSE DE DETECÇÃO DE CORES
class ColorDetector:
    def __init__(self):
        # Limites para o vermelho 
        self.red_lower = np.array([0, 136, 85])
        self.red_upper = np.array([5, 255, 255])
        self.red_lower2 = np.array([170, self.red_lower[1], self.red_lower[2]])
        self.red_upper2 = np.array([179, self.red_upper[1], self.red_upper[2]])

        # Limites para o azul 
        self.blue_lower = np.array([90, 50, 50])
        self.blue_upper = np.array([130, 255, 255])

        self.kernel = np.ones((3, 3), np.uint8)

    def detect(self, frame):
        suave = cv2.GaussianBlur(frame, (7, 7), 0)
        img_hsv = cv2.cvtColor(suave, cv2.COLOR_BGR2HSV)
        red_mask1 = cv2.inRange(img_hsv, self.red_lower, self.red_upper)
        red_mask2 = cv2.inRange(img_hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        red_dilate = cv2.dilate(red_mask, self.kernel, iterations=1)

        blue_mask = cv2.inRange(img_hsv, self.blue_lower, self.blue_upper)
        blue_dilate = cv2.dilate(blue_mask, self.kernel, iterations=1)

        color_objects = []
        for mask, color_name in [(red_dilate, "vermelho"), (blue_dilate, "azul")]:
            contours_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]

            for cnt in contours:
                if cv2.contourArea(cnt) > 1500:
                    color_objects.append({
                        "color": color_name,
                        "contour": cnt,
                        "bbox": cv2.boundingRect(cnt),
                        "processed": False
                    })
        return color_objects


# 2. CLASSE DE DETECÇÃO DE APRILTAGS (Com a inicialização original DetectorOptions)
class AprilTagDetector:
    def __init__(self):
        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)
        self.camera_params = [753.64, 755.34, 315.52, 243.88]
        self.tag_size = 0.06

    def detect(self, frame):
        if frame is None:
            return []
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)
        tags = []

        for r in results:
            pose, _, _ = self.detector.detection_pose(r, self.camera_params, self.tag_size)
            corners = r.corners.astype(int)
            tags.append({
                "id": int(r.tag_id),
                "center": (float(r.center[0]), float(r.center[1])),
                "bbox": cv2.boundingRect(corners),
                "pose_3d": (float(pose[0][3]), float(pose[1][3]), float(pose[2][3]))
            })
        return tags


# 3. MOTOR DE CLASSIFICAÇÃO
class ObjectClassifier:
    def __init__(self):
        pass

    def is_tag_inside_color(self, contour, tag_center, tag_bbox, color_bbox, margin=15):
        pt = (int(tag_center[0]), int(tag_center[1]))
        if cv2.pointPolygonTest(contour, pt, False) >= 0:
            return True
        x, y, w, h = color_bbox
        tx, ty, tw, th = tag_bbox
        return (x < tx + tw + margin and x + w > tx - margin and 
                y < ty + th + margin and y + h > ty - margin)

    def classify(self, tags, color_objects):
        classified_objects = []

        for tag in tags:
            classified_objects.append({
                "id": tag["id"],
                "color": "qualquer",
                "waypoint": "tag", 
                "bbox": tag["bbox"],
                "pose_3d": tag["pose_3d"]
            })

            for color_obj in color_objects:
                if not color_obj["processed"]:
                    if self.is_tag_inside_color(color_obj["contour"], tag["center"], tag["bbox"], color_obj["bbox"]):
                        color_obj["processed"] = True 
                        break 

        for color_obj in color_objects:
            if not color_obj["processed"]:
                x, y, w, h = color_obj["bbox"]
                cx, cy = x + w // 2, y + h // 2
                
                pos_x = (cx - 320) / 500.0
                pos_y = (cy - 240) / 500.0
                estimativa_z = 80.0 / (w + 0.1)

                classified_objects.append({
                    "id": -1,
                    "color": color_obj["color"],
                    "waypoint": "conteiner",
                    "bbox": color_obj["bbox"],
                    "pose_3d": (pos_x, pos_y, estimativa_z)
                })                
        return classified_objects


# 4. FASES DO COMPORTAMENTO
class Fase(Enum):
    IDLE = auto()       # aguardando comando; NÃO publica nada em /cmd_vel
    BUSCANDO = auto()   # translada de lado procurando a tag
    ALINHANDO = auto()  # tag avistada, alinhamento fino


# 5. NÓ ROS 2
class VisionNode(Node):

    ALVOS = {
        "cubo": {
            5.0:  (0.115,    0.685),
            10.0: (0.114278, 0.602245),
            15.0: (0.172,    0.558),
        },
        "zona": {
            5.0:  (0.0, 0.0),   # CALIBRAR
            10.0: (0.0, 0.0),   # CALIBRAR
            15.0: (0.0, 0.0),   # CALIBRAR
        },
    }

    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()
        self.color_detector = ColorDetector()
        self.tag_detector = AprilTagDetector()
        self.classifier = ObjectClassifier()

        # Parâmetros
        self.declare_parameter('search_vy', -0.1)
        self.declare_parameter('search_sweep_time', 15.0)
        self.declare_parameter('search_max_legs', 3)
        self.declare_parameter('stable_frames', 5)
        self.declare_parameter('limiar_x_memoria', 0.0)

        # Estado do comportamento
        self.fase = Fase.IDLE
        self.alvo_id = None
        self.tipo_alvo = None
        self.altura_mesa_cm = None
        self.comando_atual = None
        self.garra_ajustada = False    # Interlock com a garra
        self.frames_estaveis = 0
        self.inicio_trecho = None
        self.sentido_busca = 1          # +1 = sentido normal, -1 = inverso
        self.trechos_feitos = 0

        # Memória: local -> {id: último cam_x visto}
        self.memoria = {}
        self.local_atual = None

        self.sub = self.create_subscription(Image, '/cam/image_raw', self.image_callback, 10)
        self.sub_garra = self.create_subscription(String, '/topico_garra', self.garra_callback, 10)
        self.sub_ajuste = self.create_subscription(String, '/topico_ajuste', self.ajuste_callback, 10)
        self.pub_status = self.create_publisher(String, '/topico_garra_status', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        cv2.namedWindow("Visão do Robô", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Visão do Robô", 640, 480)

    # Parsing da mensagem
    @staticmethod
    def nome_para_id(nome):
        m = re.search(r'(\d+)$', nome.strip())
        return int(m.group(1)) if m else None

    @staticmethod
    def nome_para_tipo(nome):
        n = nome.strip().lower()
        if n.startswith('cube'):
            return 'cubo'
        if n.startswith('zone'):
            return 'zona'
        return None

    @staticmethod
    def altura_para_cm(texto_altura):
        m = re.fullmatch(r'(\d+(?:\.\d+)?)\s*cm', texto_altura.strip().lower())
        return float(m.group(1)) if m else None

    # Callbacks de Controle
    def ajuste_callback(self, msg):
        status = msg.data.strip()
        if status == 'terminou de ajustar':
            self.garra_ajustada = True
            self.get_logger().info('AVISO RECEBIDO: Garra ajustada. Visão liberada para mover o robô.')
        else:
            self.garra_ajustada = False
            self.parar_robo()
            self.get_logger().info(f'Trava ativada pela garra ({status}). Base pausada.')

    def garra_callback(self, msg):
        cmd = msg.data
        if not cmd.startswith('transladar:'):
            return

        partes = cmd.split(':')
        if len(partes) not in (3, 4):
            self.get_logger().error(f'Formato inesperado (esperado transladar:cubeX:Ncm[:local]): {cmd}')
            return

        nome_alvo = partes[1]
        altura_cm = self.altura_para_cm(partes[2])
        local = partes[3].strip() if len(partes) == 4 else 'sem_local'

        alvo = self.nome_para_id(nome_alvo)
        tipo = self.nome_para_tipo(nome_alvo)

        if alvo is None or tipo is None or altura_cm is None:
            self.get_logger().error(f'Comando inválido: {cmd}')
            return

        if altura_cm not in self.ALVOS[tipo]:
            self.get_logger().error(f'{tipo} a {altura_cm} cm sem calibração. Disponíveis: {list(self.ALVOS[tipo])}')
            return

        if local != self.local_atual:
            self.local_atual = local
            self.memoria.setdefault(local, {})

        self.comando_atual = cmd
        self.alvo_id = alvo
        self.tipo_alvo = tipo
        self.altura_mesa_cm = altura_cm
        self.frames_estaveis = 0
        self.get_logger().info(f'Comando: {cmd} -> {tipo} id {alvo}, mesa a {altura_cm} cm, local {local}')

        self.iniciar_busca()

    # Memória e Escolha de Sentido
    def registrar_tags(self, results):
        if self.fase == Fase.IDLE or self.local_atual is None:
            return
        mem = self.memoria.setdefault(self.local_atual, {})
        for item in results:
            if item["id"] >= 0:
                mem[item["id"]] = item["pose_3d"][0]

    def escolher_sentido(self):
        mem = self.memoria.get(self.local_atual, {})
        ultimo_x = mem.get(self.alvo_id)
        limiar = self.get_parameter('limiar_x_memoria').value

        if ultimo_x is None or ultimo_x > limiar:
            self.get_logger().info(f'Alvo {self.alvo_id}: sem registro ou à frente -> sentido normal')
            return 1
        self.get_logger().info(f'Alvo {self.alvo_id}: já passei por ele (x={ultimo_x:.3f}) -> voltando')
        return -1

    def iniciar_busca(self):
        self.sentido_busca = self.escolher_sentido()
        self.inicio_trecho = self.get_clock().now()
        self.trechos_feitos = 0
        self.fase = Fase.BUSCANDO

    def parar_robo(self):
        for _ in range(3):
            self.pub_cmd_vel.publish(Twist())

    def concluir_alinhamento(self):
        self.parar_robo()
        self.fase = Fase.IDLE
        self.garra_ajustada = False

        if self.tipo_alvo == 'cubo':
            self.memoria.get(self.local_atual, {}).pop(self.alvo_id, None)

        eco = String()
        eco.data = self.comando_atual
        self.pub_status.publish(eco)
        self.get_logger().info(f'Alinhado. Eco enviado em /topico_garra_status: {eco.data}')

    # Cálculo de Aproximação
    def calcular_velocidade_aproximacao(self, delta_x, delta_z, tipo, altura_cm):
        Kp_x = 8.0
        Kp_z = 2.0

        distancia_alvo_x, distancia_alvo_z = self.ALVOS[tipo][altura_cm]
        tolerancia_x = 0.01
        tolerancia_z = 0.01

        erro_frontal = delta_z - distancia_alvo_z
        erro_lateral = delta_x - distancia_alvo_x

        if abs(erro_lateral) > tolerancia_x:
            vx = Kp_x * erro_lateral
            vy = 0
            vx = max(min(vx, 0.5), -0.5)
            if abs(vx) < 0.1:
                vx = 0.1 if vx > 0 else -0.1

        elif abs(erro_frontal) > tolerancia_z:
            vy = Kp_z * erro_frontal
            vx = 0
            vy = max(min(vy, 0.5), -0.5)
            if abs(vy) < 0.1:
                vy = 0.1 if vy > 0 else -0.1

        else:
            vx = 0
            vy = 0

        return vy, -vx

    # Fases do Comportamento
    def passo_busca(self, alvo):
        if alvo is not None:
            self.get_logger().info(f'Tag {self.alvo_id} avistada! Iniciando alinhamento.')
            self.frames_estaveis = 0
            self.fase = Fase.ALINHANDO
            self.passo_alinhamento(alvo)
            return

        tempo_trecho = self.get_parameter('search_sweep_time').value
        max_trechos = self.get_parameter('search_max_legs').value
        decorrido = (self.get_clock().now() - self.inicio_trecho).nanoseconds / 1e9

        if decorrido >= tempo_trecho:
            self.trechos_feitos += 1
            if max_trechos > 0 and self.trechos_feitos >= max_trechos:
                self.get_logger().error(f'Tag {self.alvo_id} não encontrada após {self.trechos_feitos} trechos. Parando o robô.')
                self.parar_robo()
                self.fase = Fase.IDLE
                return
            self.sentido_busca *= -1
            self.inicio_trecho = self.get_clock().now()
            self.get_logger().info('Tag não encontrada neste trecho; invertendo o sentido da busca.')

        twist = Twist()
        twist.linear.y = float(self.get_parameter('search_vy').value) * self.sentido_busca
        self.pub_cmd_vel.publish(twist)

    def passo_alinhamento(self, alvo):
        if alvo is None:
            self.frames_estaveis = 0
            self.pub_cmd_vel.publish(Twist())
            return

        cam_x, _, cam_z = alvo["pose_3d"]
        v_frente, v_lado = self.calcular_velocidade_aproximacao(
            cam_x, cam_z, self.tipo_alvo, self.altura_mesa_cm)

        twist = Twist()
        twist.linear.x = float(v_frente)
        twist.linear.y = float(v_lado)
        self.pub_cmd_vel.publish(twist)

        if v_frente == 0.0 and v_lado == 0.0:
            self.frames_estaveis += 1
            if self.frames_estaveis >= self.get_parameter('stable_frames').value:
                self.concluir_alinhamento()
        else:
            self.frames_estaveis = 0

    # Callback da Câmera
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_frame = frame.copy()

            color_objects = self.color_detector.detect(frame)
            tags = self.tag_detector.detect(frame)
            results = self.classifier.classify(tags, color_objects)

            # Registra todas as tags na memória por local e ID
            self.registrar_tags(results)

            alvo_encontrado = None

            for item in results:
                x, y, w, h = item["bbox"]
                cor_bgr = (0, 0, 255) if item["color"] == "vermelho" else (255, 0, 0) if item["color"] == "azul" else (255, 255, 255)
                cv2.rectangle(display_frame, (x, y), (x + w, y + h), cor_bgr, 2)
                label = f"{item['waypoint'].upper()} | ID:{item['id']} | {item['color']}"
                cv2.putText(display_frame, label, (x, max(y - 10, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cor_bgr, 2)

                if self.alvo_id is not None and item["id"] == self.alvo_id:
                    if alvo_encontrado is None or item["waypoint"] == self.tipo_alvo:
                        alvo_encontrado = item

            # Interlock com a garra aplicado nas fases
            if self.fase == Fase.BUSCANDO:
                if self.garra_ajustada:
                    self.passo_busca(alvo_encontrado)
                else:
                    self.inicio_trecho = self.get_clock().now()  # Congela o timeout enquanto espera
                    self.parar_robo()
            elif self.fase == Fase.ALINHANDO:
                if self.garra_ajustada:
                    self.passo_alinhamento(alvo_encontrado)
                else:
                    self.parar_robo()

            status_trava = "LIBERADO" if self.garra_ajustada else "TRAVADO"
            cv2.putText(display_frame, f"FASE: {self.fase.name} [{status_trava}]", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if self.garra_ajustada else (0, 0, 255), 2)
            cv2.imshow("Visão do Robô", display_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Erro no processamento da imagem: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
