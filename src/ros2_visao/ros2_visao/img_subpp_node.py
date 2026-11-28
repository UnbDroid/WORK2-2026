from collections import Counter
from enum import Enum, auto
import re
import cv2
import math
import numpy as np
import apriltag
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# CONSTANTES DE CÂMERA / CONTÊINER
# ---------------------------------------------------------------------------
# Parâmetros intrínsecos (os mesmos usados no AprilTagDetector)
FX, FY, CX0, CY0 = 753.64, 755.34, 315.52, 243.88

# Distância do contêiner pela borda de CIMA da caixa (a base pode sair da imagem,
# então a altura da caixa não serve: ela cresce e depois diminui ao chegar perto):
#     Z = REF_V / (y_topo - Y_HORIZ)
# Y_HORIZ: constante da geometria da câmera, estimada com 4 leituras (-300).
# REF_V: define a escala de Z. CALIBRAR: no ponto ideal de parada, leia y_topo no
# log e faça REF_V = Z_ideal * (y_topo - Y_HORIZ).  Com Z_ideal = 0.602245 e
# y_topo = 249:  REF_V = 0.602245 * (249 + 300) = 330.63
Y_HORIZ = -300.0
REF_V = 330.63

# Altura mínima (em pixels) para aceitar uma massa de cor como contêiner.
# Evita que um cubo com a tag perdida num frame vire "contêiner".
# CALIBRAR: deve ser maior que a altura de um cubo e menor que a do contêiner
# na distância mais longa em que você precisa vê-lo.
H_MIN_container = 50

# Folga (px) para considerar que uma mancha de cor está dentro da caixa de uma tag.
# Cobre o alargamento da mancha pelo blur/dilate.
MARGEM_TAG_COR = 10

# Memória "já foi tag": uma mancha de cor vista com tag continua sendo tratada como
# cubo (nunca como contêiner) por este tempo (s) depois que deixa de ser vista.
TEMPO_MEMORIA_TAG = 1.0

# Na busca, contêiner cuja caixa chega a menos de BORDA_LATERAL_PX das bordas esquerda
# ou direita da imagem está cortado de lado: ainda não conta como avistado.
BORDA_LATERAL_PX = 5

# Se a borda de cima da caixa do contêiner está a no máximo estes pixels do topo da
# imagem, o topo foi cortado: o contêiner ainda está longe e só avançamos em linha reta.
Y_TOPO_CORTADO = 2

# ID negativo por cor (o comando recebe -1 ou -2)
container_IDS = {"vermelho": -1, "azul": -2}

# Mesa de precisão (PP): as tags das zonas são 101..107 (da esquerda para a direita)
# e as tags dos cubos são 1..6. O comando recebe zone1..zone7 e a visão procura a
# tag 100+N.
ZONA_TAG_BASE = 100
ZONA_N_MAX = 7
CUBO_ID_MIN, CUBO_ID_MAX = 1, 6


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


# 2. CLASSE DE DETECÇÃO DE APRILTAGS
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
        # Manchas de cor acompanhadas entre frames: {"cor", "bbox", "tem_tag", "visto_em"}
        self.trilhas = []

    def is_tag_inside_color(self, contour, tag_center, tag_bbox, color_bbox, margin=15):
        pt = (int(tag_center[0]), int(tag_center[1]))
        if cv2.pointPolygonTest(contour, pt, False) >= 0:
            return True
        x, y, w, h = color_bbox
        tx, ty, tw, th = tag_bbox
        return (x < tx + tw + margin and x + w > tx - margin and
                y < ty + th + margin and y + h > ty - margin)

    @staticmethod
    def medir_container(contour):
        """Mede o contêiner a partir da massa de cor.

        - convexHull preenche buracos na máscara (barato, sem MORPH_CLOSE)
        - centroide por momentos: mais estável que o centro do bbox
        - Z pela borda de cima da caixa (y_topo): continua visível mesmo quando
          a base do contêiner sai da imagem, e cresce sempre ao chegar perto

        Retorna (bbox, pose_3d) com pose_3d = (X, 0, Z) na mesma convenção
        das tags, ou None se a massa for pequena demais.
        """
        hull = cv2.convexHull(contour)
        m = cv2.moments(hull)
        if m["m00"] == 0:
            return None

        cx = m["m10"] / m["m00"]
        x, y, w, h = cv2.boundingRect(hull)
        if h < H_MIN_container:
            return None

        z = REF_V / (y - Y_HORIZ)
        x_m = ((cx - CX0) / FX) * z   # lateral na mesma escala de Z
        return (x, y, w, h), (x_m, 0.0, z)

    @staticmethod
    def _area_intersecao(a, b):
        ax, ay, aw, ah = a
        bx, by, bw, bh = b
        iw = min(ax + aw, bx + bw) - max(ax, bx)
        ih = min(ay + ah, by + bh) - max(ay, by)
        return iw * ih if iw > 0 and ih > 0 else 0

    def acompanhar_manchas(self, color_objects, agora):
        """Acompanha as manchas de cor de um frame para o outro (mesma cor e caixas que
        se sobrepõem). Mancha vista com tag ganha a marca "tem_tag"; se depois ela
        aparecer sem tag (tag perdida no frame), continua valendo como cubo e não vira
        contêiner. A marca é esquecida TEMPO_MEMORIA_TAG s depois que a mancha some."""
        self.trilhas = [t for t in self.trilhas if agora - t["visto_em"] <= TEMPO_MEMORIA_TAG]

        atualizacoes = []   # (trilha ou None, mancha, estava_com_tag)
        for color_obj in color_objects:
            if color_obj.get("fantasma"):   # preto da tag, não é objeto
                continue
            melhor, area_melhor = None, 0
            for t in self.trilhas:
                if t["cor"] != color_obj["color"]:
                    continue
                area = self._area_intersecao(t["bbox"], color_obj["bbox"])
                if area > area_melhor:
                    melhor, area_melhor = t, area
            atualizacoes.append((melhor, color_obj, color_obj["processed"]))

        novas = []
        for trilha, color_obj, com_tag in atualizacoes:
            if trilha is None:
                novas.append({"cor": color_obj["color"], "bbox": color_obj["bbox"],
                              "tem_tag": com_tag, "visto_em": agora})
                continue
            if trilha["tem_tag"] and not com_tag:
                color_obj["processed"] = True   # já foi visto com tag: é cubo
            trilha["tem_tag"] = trilha["tem_tag"] or com_tag
            trilha["bbox"] = color_obj["bbox"]
            trilha["visto_em"] = agora
        self.trilhas.extend(novas)

    def classify(self, tags, color_objects, agora=0.0):
        classified_objects = []

        # Mancha de cor que cabe inteira dentro da caixa de uma tag detectada é o preto
        # da própria tag lido como cor (ex.: azul): não é objeto, então é ignorada.
        for color_obj in color_objects:
            x, y, w, h = color_obj["bbox"]
            for tag in tags:
                tx, ty, tw, th = tag["bbox"]
                if (x >= tx - MARGEM_TAG_COR and y >= ty - MARGEM_TAG_COR and
                        x + w <= tx + tw + MARGEM_TAG_COR and y + h <= ty + th + MARGEM_TAG_COR):
                    color_obj["processed"] = True
                    color_obj["fantasma"] = True
                    break

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
                    if self.is_tag_inside_color(color_obj["contour"], tag["center"],
                                                tag["bbox"], color_obj["bbox"]):
                        color_obj["processed"] = True
                        break

        # Manchas que já foram vistas com tag (neste ou em frames anteriores) são cubos
        self.acompanhar_manchas(color_objects, agora)

        # Massas de cor sem tag dentro = contêineres
        for color_obj in color_objects:
            if color_obj["processed"]:
                continue

            medida = self.medir_container(color_obj["contour"])
            if medida is None:
                continue
            bbox, pose_3d = medida

            classified_objects.append({
                "id": container_IDS[color_obj["color"]],
                "color": color_obj["color"],
                "waypoint": "container",
                "bbox": bbox,
                "h_px": bbox[3],
                "pose_3d": pose_3d
            })
        return classified_objects


# 4. FASES DO COMPORTAMENTO
class Fase(Enum):
    IDLE = auto()       # aguardando comando; NÃO publica nada em /cmd_vel
    BUSCANDO = auto()   # translada de lado procurando a tag
    ALINHANDO = auto()  # tag avistada, alinhamento fino
    INSPECIONANDO = auto()  # parado em frente a uma zona, lendo qual cubo está embaixo
    GUARDANDO = auto()      # parado entre dois places na mesma mesa, gravando as tags à vista na memória


# 5. NÓ ROS 2
class VisionNode(Node):

    ALVOS = {
        "cubo": {
            5.0:  (0.233, 0.784),
            10.0: (0.145, 0.714),
            15.0: (0.195, 0.649),
        },
        "zona": {
            5.0:  (0.0, 0.0),   # CALIBRAR
            10.0: (0.0, 0.0),   # CALIBRAR
            15.0: (0.0, 0.0),   # CALIBRAR
        },
        "container": {
            5.0:  (0.158,0.598),
            10.0: (0.148, 0.720),
            15.0: (0.2124 , 0.8413),
        },
        "empilhamento": {
            5.0:  (0.0, 0.0),   # CALIBRAR
            10.0: (0.208, 0.719),
            15.0: (0.0, 0.0),   # CALIBRAR
        },
    }

    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()
        self.color_detector = ColorDetector()
        self.tag_detector = AprilTagDetector()
        self.classifier = ObjectClassifier()

        # Parâmetros (ros2 run ... --ros-args -p search_vy:=-0.3)
        self.declare_parameter('search_vy', -0.1)           # VELOCIDADE (m/s) da translação na busca; o sinal define o sentido "normal"
        # TEMPO (s) do 1º trecho da busca (ponto de partida -> uma ponta da mesa); os seguintes
        # duram o dobro. Busca esgotada leva T + 2T + 2T = 5T: precisa ficar abaixo do timeout
        # das ações (TIMEOUT_VISAO = 150 s na rotina PP). Com 10 s: 50 s e 1 m por trecho a 0.1 m/s.
        self.declare_parameter('search_sweep_time', 10.0)
        self.declare_parameter('frames_olhar_inicio', 5)    # frames parado olhando antes de escolher o sentido da busca
        self.declare_parameter('search_max_legs', 3)        # nº máximo de trechos antes de desistir (0 = varre para sempre)
        self.declare_parameter('stable_frames', 5)          # frames seguidos dentro da tolerância para concluir o alinhamento
        self.declare_parameter('frames_para_avistar', 3)    # frames seguidos vendo a tag antes de iniciar o alinhamento
        self.declare_parameter('frames_perdida_limite', 15) # frames sem ver a tag no alinhamento antes de voltar a buscar
        self.declare_parameter('dist_min_memoria', 0.05)    # (m) abaixo disso a tag lembrada é considerada "aqui perto"
        self.declare_parameter('suavizacao_container', 0.4)  # 0..1, menor = mais suave
        self.declare_parameter('velocidade_cega_container', 0.3)  # velocidade de avanço enquanto o topo do contêiner está cortado
        self.declare_parameter('inspecao_frames', 10)       # frames (com a tag da zona visível) usados para decidir o inspecionar
        self.declare_parameter('inspecao_votos_min', 2)     # nº mínimo de frames vendo o MESMO cubo para aceitar; senão = vazio
        self.declare_parameter('inspecao_timeout', 10.0)    # (s) tempo total do inspecionar; passou disso sem juntar os frames, desiste e não responde
        self.declare_parameter('pareamento_tol_x', 1.0)     # tolerância lateral cubo x zona, em larguras da tag da zona
        self.declare_parameter('suavizacao_tag', 0.5)
        # Ganhos do controle PI de aproximação (ki = 0 volta a ser só P, igual antes)
        self.declare_parameter('kp_x', 3.0)    # proporcional lateral
        self.declare_parameter('kp_z', 3.0)    # proporcional frontal
        self.declare_parameter('ki_x', 2.0)    # integral lateral (1/s)
        self.declare_parameter('ki_z', 2.0)    # integral frontal (1/s)
        self.declare_parameter('i_max', 0.05)  # (m/s) contribuição máxima do termo integral (anti-windup)
        # Guardar (pedido pelo place_at_location com guardar:<local> entre dois places na mesma mesa)
        self.declare_parameter('guardar_atraso', 0.5)  # (s) espera inicial sem gravar (garra recolhendo, imagem assentando)
        self.declare_parameter('guardar_tempo', 2.0)   # (s) tempo observando e gravando as tags, depois do atraso
        # Calibração: com log_tags = true, a cada calib_frames frames imprime a média e o
        # desvio de X e Z de cada tag à vista, em QUALQUER fase (inclusive parado em IDLE).
        # Pode ligar com o nó rodando:  ros2 param set /vision_node log_tags true
        self.declare_parameter('log_tags', False)
        self.declare_parameter('calib_frames', 30)

        # Estado do comportamento
        self.fase = Fase.IDLE
        self.alvo_id = None
        self.tipo_alvo = None
        self.altura_mesa_cm = None
        self.comando_atual = None
        self.frames_estaveis = 0
        self.frames_vistos = 0
        self.frames_perdidos = 0
        self.ultimo_erro_lateral = 0.0
        self.inicio_trecho = None
        self.sentido_busca = 1          # +1 = sentido normal (o de search_vy), -1 = inverso
        self.trechos_feitos = 0

        # Filtro exponencial do contêiner (X, Z suavizados)
        self.x_filt = None
        self.z_filt = None
        self.vx_ant = 0.0
        self.vy_ant = 0.0

        # Estado do PI: integral do erro (m*s) por eixo e instante do último cálculo
        self.integral = {"x": 0.0, "z": 0.0}
        self.t_pi_ant = None

        # Mesa PP: zonas visíveis no último frame e estado do inspecionar
        self.zonas_visiveis = []
        self.zona_inspecionada = None   # id da tag da zona (101..107)
        self.nome_zona_inspecao = None  # nome recebido no comando (ex.: zone1), devolvido na resposta
        self.votos_inspecao = []
        self.inicio_inspecao = None

        # Guardar: o place_at_location manda guardar:<local> em /topico_garra quando faz outro
        # place na mesma mesa. O robô fica parado, grava as tags à vista na memória e responde
        # guardar:<local> em /topico_garra_status.
        self.local_guardar = None
        self.inicio_guardar = None
        self.tags_guardadas = set()

        # Início da busca: alguns frames parado olhando (a câmera acabou de ligar) para saber
        # em frente a qual tag o robô está antes de escolher o sentido.
        self.frames_olhar = 0

        # Memória: local -> lista dos ids das tags na ordem em que estão na mesa,
        # da ESQUERDA para a DIREITA. Ex.: [1, 2, 3] = o 1 está à esquerda do 2, que está
        # à esquerda do 3. Só a ordem importa, não a distância. A lista de cada mesa é
        # mantida quando o robô sai e volta (a ordem continua valendo).
        self.memoria = {}
        self.local_atual = None
        # Tag que está (ou estava por último) na frente da câmera: é "onde o robô está"
        # dentro da lista acima.
        self.ultima_tag = None
        # Posição lateral estimada (eixo de y_est) onde cada tag foi vista por último.
        # Usada SÓ para decidir de que lado da última tag uma tag nova (vista sozinha) fica.
        self.y_tags = {}

        # Posição lateral estimada do robô (m, +y = esquerda), integrando a velocidade lateral
        # de TUDO que é publicado em /cmd_vel (este nó, a translação do place_at_location, o
        # ajuste...). Antes só contava o que este nó publicava. Zera ao trocar de mesa.
        self.y_est = 0.0
        self.vy_ultimo = 0.0
        self.t_ultimo_cmd = None

        self.cubo_identificado_enviado = False #mudei

        # Calibração: amostras (X, Z) por id de tag, para a média do log_tags
        self.calib_amostras = {}

        self.sub = self.create_subscription(Image, '/cam/image_raw', self.image_callback, 10)
        self.sub_garra = self.create_subscription(String, '/topico_garra', self.garra_callback, 10)
        self.pub_garra = self.create_publisher(String, '/topico_garra', 10) #mudei
        self.pub_status = self.create_publisher(String, '/topico_garra_status', 10)
        self.pub_deteccao = self.create_publisher(String, '/topico_deteccao', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer_seguranca_guardar = self.create_timer(0.2, self.timer_guardar)

        cv2.namedWindow("Visão do Robô", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Visão do Robô", 640, 480)

    # Parsing da mensagem
    @staticmethod
    def nome_para_id(nome):
        """cube5 -> 5 | zone2 -> 2 | container-1 -> -1 | container-2 -> -2"""
        n = nome.strip().lower()
        m = re.search(r'(-?\d+)$', n)
        return int(m.group(1)) if m else None

    @staticmethod
    def nome_para_tipo(nome):
        n = nome.strip().lower()
        if n.startswith('cube'):
            return 'cubo'
        if n.startswith('zone'):
            return 'zona'
        if n.startswith('container'):
            return 'container'
        if n.startswith('empilhamento'):
            return 'empilhamento'
        return None

    @staticmethod
    def altura_para_cm(texto_altura):
        m = re.fullmatch(r'(\d+(?:\.\d+)?)\s*(?:cm)?', texto_altura.strip().lower())
        return float(m.group(1)) if m else None

    def resetar_filtro(self):
        self.x_filt = None
        self.z_filt = None

    def resetar_rampa(self, vx=0.0, vy=0.0):
        # Ponto de partida da rampa. Na passagem busca -> alinhamento, vx recebe a
        # velocidade lateral da busca, para o robô não dar um tranco ao começar a alinhar.
        self.vx_ant = float(vx)
        self.vy_ant = float(vy)
        # Todo lugar que reinicia a rampa também começa um alinhamento novo (ou encerra
        # um): o integral acumulado do alvo anterior não vale mais.
        self.resetar_integral()

    def resetar_integral(self):
        self.integral = {"x": 0.0, "z": 0.0}
        self.t_pi_ant = None

    def log_deteccao(self, texto, no_topico=True):
        """Mostra o log no terminal e, se no_topico, publica o MESMO texto em /topico_deteccao."""
        self.get_logger().info(texto)
        if no_topico:
            self.pub_deteccao.publish(String(data=texto))

    # Callback de comando (único canal de controle: /topico_garra)
    def garra_callback(self, msg):
        cmd = msg.data

        if cmd == 'guardar' or cmd.startswith('guardar:'):
            self.receber_guardar(cmd)
            return

        if cmd.startswith('inspecionar:'):
            self.iniciar_inspecao(cmd)
            return
        if cmd.startswith('impossivel'):
            if self.fase in (Fase.BUSCANDO, Fase.ALINHANDO):
                self.get_logger().warning(
                    f'(listener) [IMPOSSÍVEL] "{cmd}" recebido na fase {self.fase.name} (alvo {self.comando_atual}). '
                    f'Parei de buscar/alinhar e vou responder como se o alvo tivesse sido pego.')
                self.concluir_alinhamento(fingido=True)
                self.get_logger().warning(
                    f'(listener) [IMPOSSÍVEL] Eco enviado (alinhamento FINGIDO, nada foi pego): {self.comando_atual}. '
                    f'A garra segue para o próximo passo.')
            else:
                self.get_logger().info(
                    f'(listener) [IMPOSSÍVEL] "{cmd}" ignorado: robô na fase {self.fase.name}, não há alinhamento para encerrar.')
            return
        if not cmd.startswith('transladar:'):
            return

        partes = cmd.split(':')
        if len(partes) not in (3, 4):
            self.get_logger().error(f'(listener) Formato inesperado (esperado transladar:cubeX:Ncm[:local]): {cmd}')
            return

        nome_alvo = partes[1]
        altura_cm = self.altura_para_cm(partes[2])
        local = partes[3].strip() if len(partes) == 4 else 'sem_local'

        alvo = self.nome_para_id(nome_alvo)
        tipo = self.nome_para_tipo(nome_alvo)

        if alvo is None or tipo is None or altura_cm is None:
            self.get_logger().error(f'(listener) Comando inválido: {cmd}')
            return

        if tipo == 'container' and alvo not in container_IDS.values():
            self.get_logger().error(
                f'(listener) Contêiner inválido (use -1 = vermelho, -2 = azul): {cmd}')
            return

        if tipo != 'container' and alvo < 0:
            self.get_logger().error(f'(listener) ID negativo só é válido para contêiner: {cmd}')
            return

        if tipo == 'zona':
            # zoneN (N = 1..7) -> tag 100+N
            if not 1 <= alvo <= ZONA_N_MAX:
                self.get_logger().error(f'(listener) Zona inválida (use zone1..zone{ZONA_N_MAX}): {cmd}')
                return
            alvo = ZONA_TAG_BASE + alvo

        if altura_cm not in self.ALVOS[tipo]:
            self.get_logger().error(f'(listener) {tipo} a {altura_cm} cm sem calibração. Disponíveis: {list(self.ALVOS[tipo])}')
            return

        if self.ALVOS[tipo][altura_cm] == (0.0, 0.0):
            self.get_logger().error(f'(listener) {tipo} a {altura_cm} cm ainda não calibrado (alvo 0, 0). Comando ignorado.')
            return

        if self.fase == Fase.GUARDANDO:
            # Um transladar no meio do guardar quer dizer que o plano já seguiu (ex.: o
            # place_at_location desistiu de esperar): o guardar é cancelado e o transladar
            # é atendido normalmente, para nada ficar travado.
            self.get_logger().warning(
                f'(listener) {cmd} recebido durante o guardar de {self.local_guardar}; guardar cancelado.')
            self.fase = Fase.IDLE

        if local != self.local_atual:
            self.mudar_local(local)

        self.comando_atual = cmd
        self.alvo_id = alvo
        self.tipo_alvo = tipo
        self.altura_mesa_cm = altura_cm
        self.frames_estaveis = 0
        self.resetar_filtro()
        self.resetar_rampa()
        self.get_logger().info(f'(listener) Comando: {cmd} -> {tipo} id {alvo}, mesa a {altura_cm} cm, local {local}')

        self.iniciar_busca()

    def mudar_local(self, local):
        """Mudou de mesa. A ORDEM das tags desta mesa (se o robô já esteve aqui) continua
        valendo e é mantida. O que zera é o referencial de posição: a posição lateral
        estimada, as posições das tags e a tag "em frente" (o robô chegou num ponto novo)."""
        self.local_atual = local
        self.memoria.setdefault(local, [])
        self.ultima_tag = None
        self.y_tags = {}
        self.y_est = 0.0
        if self.memoria[local]:
            self.get_logger().info(f'(listener) De volta a {local}; ordem lembrada: {self.memoria[local]}')

    # Publicação de velocidade
    def publicar_cmd(self, twist):
        # A posição lateral (y_est) é integrada em cmd_vel_callback, que recebe também
        # estas mensagens: não integrar aqui, senão conta duas vezes.
        self.pub_cmd_vel.publish(twist)

    def cmd_vel_callback(self, twist):
        """Integra a velocidade lateral de qualquer nó que publique em /cmd_vel."""
        agora = self.get_clock().now()
        if self.t_ultimo_cmd is not None:
            dt = (agora - self.t_ultimo_cmd).nanoseconds / 1e9
            if dt < 0.5:  # ignora lacunas (ninguém publicando: robô parado)
                self.y_est += self.vy_ultimo * dt
        self.t_ultimo_cmd = agora
        self.vy_ultimo = twist.linear.y

    def parar_robo(self):
        for _ in range(3):
            self.publicar_cmd(Twist())

    # Memória e escolha de sentido
    def registrar_tags(self, results, forcar=False):
        """Guarda a ORDEM (esquerda -> direita) das tags vistas neste local.

        - Tag nova vista junto com uma tag já conhecida no mesmo frame: entra do lado
          certo dela, pela posição na imagem.
        - Tag nova vista sozinha (nenhuma conhecida no frame): entra logo ao lado da última
          tag vista, do lado em que ela está em relação a essa última tag (comparando as
          posições laterais estimadas). Funciona mesmo se o robô inverteu o sentido no meio.
        Também atualiza ultima_tag (a tag mais perto do centro da imagem).
        Só roda na busca e no alinhamento: em IDLE/INSPECIONANDO o comando atual pode ser
        de outro local (inspecionar não diz o local) e estragaria a lista deste.
        forcar=True: chamado pelo guardar, que já garantiu que local_atual é o local certo."""
        if (self.fase not in (Fase.BUSCANDO, Fase.ALINHANDO) and not forcar) or self.local_atual is None:
            return
        ordem = self.memoria.setdefault(self.local_atual, [])

        # Tags deste frame, da esquerda para a direita na imagem (x da câmera + = direita).
        # A mesma tag pode aparecer duas vezes no frame: fica só a primeira.
        vistas = []
        for r in sorted((r for r in results if r["id"] >= 0), key=lambda r: r["pose_3d"][0]):
            if all(v["id"] != r["id"] for v in vistas):
                vistas.append(r)
        if not vistas:
            return
        ids = [r["id"] for r in vistas]
        tamanho_antes = len(ordem)

        # Posição lateral estimada de cada tag deste frame (+ = esquerda; x da câmera + = direita)
        y_frame = {r["id"]: self.y_est - r["pose_3d"][0] for r in vistas}

        conhecidas = [i for i, t in enumerate(ids) if t in ordem]
        if conhecidas:
            # As novas à esquerda da primeira conhecida entram logo antes dela
            primeira = conhecidas[0]
            pos = ordem.index(ids[primeira])
            for t in ids[:primeira]:
                ordem.insert(pos, t)
                pos += 1
            # As demais novas entram logo depois da tag à esquerda delas no frame
            for i in range(primeira + 1, len(ids)):
                if ids[i] not in ordem:
                    ordem.insert(ordem.index(ids[i - 1]) + 1, ids[i])
        elif not ordem:
            ordem.extend(ids)
        elif self.ultima_tag not in ordem or self.ultima_tag not in self.y_tags:
            # A mesa já tem ordem lembrada (de uma visita anterior), mas nada deste frame é
            # conhecido e não há referência de onde o robô está: sem como saber o lado,
            # não insere. Estas tags entram quando aparecerem junto com uma conhecida.
            pass
        else:
            y_ult = self.y_tags[self.ultima_tag]
            a_esquerda = [t for t in ids if y_frame[t] > y_ult]    # já em ordem esq -> dir
            a_direita = [t for t in ids if y_frame[t] <= y_ult]
            pos = ordem.index(self.ultima_tag)
            ordem[pos:pos] = a_esquerda                             # logo à esquerda da última
            pos = ordem.index(self.ultima_tag)
            ordem[pos + 1:pos + 1] = a_direita                      # logo à direita da última

        if len(ordem) != tamanho_antes:
            self.get_logger().info(
                f'(listener) Ordem das tags em {self.local_atual}: {ordem}')

        self.y_tags.update(y_frame)
        self.ultima_tag = min(vistas, key=lambda r: abs(r["pose_3d"][0]))["id"]

    def sentido_para(self, esquerda):
        """Converte 'ir para a esquerda/direita' no sentido (+1/-1) usado pela busca."""
        sentido_normal_e_esquerda = self.get_parameter('search_vy').value > 0
        return 1 if esquerda == sentido_normal_e_esquerda else -1

    def sentido_pela_ordem_das_zonas(self):
        """Zonas estão em ordem crescente da esquerda para a direita. Se a zona pedida
        não está à vista mas outras estão, a pedida fica do lado de fora do grupo visível.
        Devolve o sentido (+1/-1) ou None se não der para decidir."""
        visiveis = self.zonas_visiveis
        if not visiveis or self.alvo_id in visiveis:
            return None
        if self.alvo_id > max(visiveis):
            esquerda = False
        elif self.alvo_id < min(visiveis):
            esquerda = True
        else:
            return None
        sentido = self.sentido_para(esquerda)
        self.get_logger().info(
            f'(listener) Zona {self.alvo_id - ZONA_TAG_BASE}: vendo {sorted(visiveis)} -> '
            f'vou para a {"esquerda" if esquerda else "direita"} '
            f'(sentido {"normal" if sentido == 1 else "inverso"})')
        return sentido

    def escolher_sentido(self):
        if self.tipo_alvo == 'zona':
            sentido = self.sentido_pela_ordem_das_zonas()
            if sentido is not None:
                return sentido

        ordem = self.memoria.get(self.local_atual, [])

        if self.alvo_id not in ordem or self.ultima_tag not in ordem:
            self.get_logger().info(f'(listener) Alvo {self.alvo_id}: sem registro -> sentido normal')
            return 1

        if self.alvo_id == self.ultima_tag:
            self.get_logger().info(f'(listener) Alvo {self.alvo_id}: é a última tag vista (aqui na frente) -> sentido normal')
            return 1

        # Na lista (esquerda -> direita), alvo antes de onde estou = alvo à esquerda
        esquerda = ordem.index(self.alvo_id) < ordem.index(self.ultima_tag)
        sentido = self.sentido_para(esquerda)
        self.get_logger().info(
            f'(listener) Alvo {self.alvo_id}: ordem {ordem}, estou na {self.ultima_tag} -> vou para a '
            f'{"esquerda" if esquerda else "direita"} (sentido {"normal" if sentido == 1 else "inverso"})')
        return sentido

    def iniciar_busca(self):
        # O sentido só é escolhido depois de alguns frames parado olhando (ver passo_busca):
        # a câmera acabou de ligar e o robô pode ter chegado agora a esta mesa, então ainda
        # não se sabe em frente a qual tag ele está.
        self.frames_olhar = int(self.get_parameter('frames_olhar_inicio').value)
        self.sentido_busca = 1
        self.inicio_trecho = self.get_clock().now()
        self.trechos_feitos = 0
        self.frames_vistos = 0
        self.fase = Fase.BUSCANDO
        if self.frames_olhar <= 0:
            self.sentido_busca = self.escolher_sentido()

    def concluir_alinhamento(self, fingido=False):
        """Para o robô, volta para IDLE e manda o eco do comando em /topico_garra_status.

        fingido=True: o alvo NÃO foi alcançado (comando 'impossivel' ou busca esgotada).
        O eco é o mesmo, para a garra e o plano seguirem, mas a memória de ordem não é
        alterada: o robô não está na frente do alvo."""
        self.parar_robo()
        self.fase = Fase.IDLE
        self.resetar_filtro()
        self.resetar_rampa()

        # Alinhado de verdade = o robô está na frente do alvo. O alvo continua na lista
        # mesmo depois de pego: o lugar dele serve de referência para achar os outros.
        if not fingido and self.alvo_id is not None and self.alvo_id >= 0:
            self.ultima_tag = self.alvo_id
            self.y_tags.setdefault(self.alvo_id, self.y_est)

        eco = String()
        eco.data = self.comando_atual
        self.pub_status.publish(eco)
        if fingido:
            self.get_logger().warning(f'(listener) Alinhamento FINGIDO. Eco enviado em /topico_garra_status: {eco.data}')
        else:
            self.get_logger().info(f'(listener) Alinhado. Eco enviado em /topico_garra_status: {eco.data}')

    # Guardar: pedido pelo place_at_location (guardar:<local>) entre dois places na mesma
    # mesa. Parado, grava na memória as tags à vista e responde guardar:<local>.
    # Toda chegada de guardar tem resposta (mesmo quando não dá para guardar), para o
    # place_at_location nunca ficar esperando.
    def receber_guardar(self, cmd):
        partes = cmd.split(':', 1)
        local = partes[1].strip() if len(partes) == 2 and partes[1].strip() else self.local_atual

        if local is None:
            self.get_logger().error(f'(listener) Guardar sem local e nenhum transladar recebido ainda: {cmd}')
            self.pub_status.publish(String(data=cmd))
            return
        if self.fase != Fase.IDLE:
            self.get_logger().error(
                f'(listener) Robô ocupado (fase {self.fase.name}); guardar não feito, só respondido: {cmd}')
            self.pub_status.publish(String(data=cmd))
            return
        self.iniciar_guardar(local)

    def iniciar_guardar(self, local):
        """Para o robô e começa a observar."""
        if local != self.local_atual:
            self.mudar_local(local)
        self.parar_robo()
        self.resetar_filtro()
        self.resetar_rampa()
        self.local_guardar = local
        self.inicio_guardar = self.get_clock().now()
        self.tags_guardadas = set()
        self.fase = Fase.GUARDANDO
        self.get_logger().info(f'(listener) Guardando a memória de {local}...')

    def timer_guardar(self):
        """Segurança: se a câmera parar de mandar frames no meio do guardar, ele termina
        mesmo assim (com o que tiver gravado), em vez de ficar preso em GUARDANDO."""
        if self.fase != Fase.GUARDANDO:
            return
        decorrido = (self.get_clock().now() - self.inicio_guardar).nanoseconds / 1e9
        limite = (float(self.get_parameter('guardar_atraso').value)
                  + float(self.get_parameter('guardar_tempo').value) + 2.0)
        if decorrido > limite:
            self.get_logger().warning('(listener) Guardar sem frames da câmera; encerrando com o que foi visto.')
            self.concluir_guardar()

    def passo_guardar(self, results):
        decorrido = (self.get_clock().now() - self.inicio_guardar).nanoseconds / 1e9
        atraso = float(self.get_parameter('guardar_atraso').value)
        if decorrido < atraso:
            return   # espera a garra sair da frente e a imagem assentar

        try:
            self.registrar_tags(results, forcar=True)
        except Exception as e:
            self.get_logger().error(f'(listener) Erro ao registrar a ordem das tags no guardar: {e}')
        self.tags_guardadas.update(r["id"] for r in results if r["id"] >= 0)

        if decorrido >= atraso + float(self.get_parameter('guardar_tempo').value):
            self.concluir_guardar()

    def concluir_guardar(self):
        self.fase = Fase.IDLE
        ordem = self.memoria.get(self.local_guardar, [])
        cubos = sorted(t for t in self.tags_guardadas if CUBO_ID_MIN <= t <= CUBO_ID_MAX)

        eco = String()
        eco.data = f'guardar:{self.local_guardar}'
        self.pub_status.publish(eco)
        self.log_deteccao(
            f'(listener) Guardar concluído em {self.local_guardar}: cubos vistos {cubos}, '
            f'ordem na memória {ordem}. Eco em /topico_garra_status: {eco.data}')

    # Inspecionar: qual cubo está embaixo da zona em frente?
    def iniciar_inspecao(self, cmd):
        partes = cmd.split(':')
        if len(partes) != 2:
            self.get_logger().error(f'(listener) Formato inesperado (esperado inspecionar:zoneN): {cmd}')
            return

        nome = partes[1].strip()
        n = self.nome_para_id(nome)
        if self.nome_para_tipo(nome) != 'zona' or n is None or not 1 <= n <= ZONA_N_MAX:
            self.get_logger().error(f'(listener) Inspeção inválida (use inspecionar:zone1..zone{ZONA_N_MAX}): {cmd}')
            return

        if self.fase != Fase.IDLE:
            self.get_logger().error(f'(listener) Robô ocupado (fase {self.fase.name}); inspeção ignorada: {cmd}')
            return

        self.zona_inspecionada = ZONA_TAG_BASE + n
        self.nome_zona_inspecao = nome
        self.votos_inspecao = []
        self.inicio_inspecao = self.get_clock().now()
        self.fase = Fase.INSPECIONANDO
        self.get_logger().info(f'(listener) Inspecionando {nome} (tag {self.zona_inspecionada})')

    def cubo_abaixo(self, zona_tag, results):
        """Id do cubo (tag 1..6) logo ABAIXO da tag da zona na imagem, ou None.
        Abaixo = centro mais para baixo que o da zona e deslocamento lateral até
        pareamento_tol_x larguras da tag da zona. Havendo mais de um, o mais alinhado."""
        zx, zy, zw, zh = zona_tag["bbox"]
        zcx, zcy = zx + zw / 2.0, zy + zh / 2.0
        tol = float(self.get_parameter('pareamento_tol_x').value) * zw

        melhor = None
        for r in results:
            if not CUBO_ID_MIN <= r["id"] <= CUBO_ID_MAX:
                continue
            x, y, w, h = r["bbox"]
            cx, cy = x + w / 2.0, y + h / 2.0
            if cy <= zcy:
                continue
            dx = abs(cx - zcx)
            if dx > tol:
                continue
            if melhor is None or dx < melhor[0]:
                melhor = (dx, r["id"])
        return melhor[1] if melhor else None

    def passo_inspecao(self, results):
        """Parado em frente à zona: junta alguns frames e decide vazio / certo / cubeN."""
        decorrido = (self.get_clock().now() - self.inicio_inspecao).nanoseconds / 1e9
        if decorrido > float(self.get_parameter('inspecao_timeout').value):
            self.get_logger().error(
                f'(listener) Tag da zona {self.zona_inspecionada} não apareceu em {decorrido:.0f} s; inspeção cancelada, sem resposta.')
            self.fase = Fase.IDLE
            return

        zona_tag = next((r for r in results if r["id"] == self.zona_inspecionada), None)
        if zona_tag is None:
            return   # sem a tag da zona não dá para dizer o que está embaixo dela: frame não conta

        self.votos_inspecao.append(self.cubo_abaixo(zona_tag, results))
        if len(self.votos_inspecao) < int(self.get_parameter('inspecao_frames').value):
            return

        votos = Counter(v for v in self.votos_inspecao if v is not None)
        votos_min = int(self.get_parameter('inspecao_votos_min').value)
        if votos and votos.most_common(1)[0][1] >= votos_min:
            cubo = votos.most_common(1)[0][0]
            resultado = 'certo' if cubo == self.zona_inspecionada - ZONA_TAG_BASE else f'cube{cubo}'
        else:
            resultado = 'vazio'

        eco = String()
        eco.data = f'inspecionar:{self.nome_zona_inspecao}:{resultado}'
        self.pub_status.publish(eco)
        self.log_deteccao(f'(listener) Inspeção concluída (votos {dict(votos)}). Resposta em /topico_garra_status: {eco.data}')
        self.fase = Fase.IDLE

    # Cálculo de Aproximação
    def calcular_velocidade_aproximacao(self, delta_x, delta_z, tipo, altura_cm):
        Kp_x = float(self.get_parameter('kp_x').value)
        Kp_z = float(self.get_parameter('kp_z').value)
        Ki_x = float(self.get_parameter('ki_x').value)
        Ki_z = float(self.get_parameter('ki_z').value)
        I_MAX = float(self.get_parameter('i_max').value)
        V_MAX_X = 0.125
        V_MAX_Z = 0.25
        V_MIN = 0.04
        DV = 0.02
        DT_MAX = 0.2   # (s) dt maior que isso (frame atrasado, tag sumiu) não entra inteiro no integral

        distancia_alvo_x, distancia_alvo_z = self.ALVOS[tipo][altura_cm]
        tolerancia_x = 0.01
        tolerancia_z = 0.01

        erro_frontal = delta_z - distancia_alvo_z
        erro_lateral = delta_x - distancia_alvo_x

        # Tempo desde o último cálculo (o integral é erro * tempo, não erro * frame)
        agora = self.get_clock().now()
        if self.t_pi_ant is None:
            dt = 0.0
        else:
            dt = min((agora - self.t_pi_ant).nanoseconds / 1e9, DT_MAX)
        self.t_pi_ant = agora

        def eixo(erro, nome, kp, ki, v_max, tolerancia):
            # Dentro da tolerância: parado neste eixo, e o integral é zerado para não
            # empurrar o robô para fora do ponto depois que ele chegou
            if abs(erro) < tolerancia:
                self.integral[nome] = 0.0
                return 0.0

            i_ant = self.integral[nome]
            # Erro trocou de sinal (passou do ponto): o integral antigo empurraria mais
            # ainda para o lado errado, então recomeça do zero
            if i_ant * erro < 0:
                i_ant = 0.0

            i_novo = i_ant + erro * dt
            # Anti-windup 1: o termo integral sozinho nunca passa de I_MAX
            if ki > 0:
                lim = I_MAX / ki
                i_novo = max(min(i_novo, lim), -lim)
            # Anti-windup 2: se a saída já está saturada e o erro empurra para o mesmo
            # lado, não acumula (acumular aí só causaria ultrapassagem depois)
            v_bruto = kp * erro + ki * i_novo
            if abs(v_bruto) > v_max and v_bruto * erro > 0:
                i_novo = i_ant
            self.integral[nome] = i_novo

            # PI, com velocidade mínima (vence o atrito) e saturação
            v = kp * erro + ki * i_novo
            v = math.copysign(max(abs(v), V_MIN), v)
            return max(min(v, v_max), -v_max)

        def rampa(v_novo, v_ant):
            # Limita a aceleração (e a troca de sentido) a DV por frame; frear é imediato
            if abs(v_novo) > abs(v_ant) or v_novo * v_ant < 0:
                return v_ant + max(min(v_novo - v_ant, DV), -DV)
            return v_novo

        vx = rampa(eixo(erro_lateral, "x", Kp_x, Ki_x, V_MAX_X, tolerancia_x), self.vx_ant)
        vy = rampa(eixo(erro_frontal, "z", Kp_z, Ki_z, V_MAX_Z, tolerancia_z), self.vy_ant)
        self.vx_ant, self.vy_ant = vx, vy

        print(f'(listener) Velocidade final: vx={vx:.3f} e vy={vy:.3f}  A coordenada X atual: {delta_x:.3f}, o erro lateral atual é:{erro_lateral:.3f} , A coordenada Y atual: {delta_z:.3f}, O erro frontal atual é:{erro_frontal:.3f}'
              f'  | I_x={Ki_x * self.integral["x"]:.3f} I_z={Ki_z * self.integral["z"]:.3f} m/s')

        # (v_frente, v_lado): frente = linear.x, lado = linear.y (+y = esquerda)
        return vy, -vx

    # Fases do Comportamento
    def passo_busca(self, alvo):
        """Fase 1: translada de lado; inverte o sentido a cada trecho sem achar a tag."""
        if self.frames_olhar > 0:
            # Começo da busca: parado alguns frames. registrar_tags (que roda antes deste
            # passo) atualiza a tag em frente; com ela, a memória diz para que lado ir.
            self.frames_olhar -= 1
            if alvo is None and self.frames_olhar > 0:
                return
            self.frames_olhar = 0
            self.sentido_busca = self.escolher_sentido()
            self.inicio_trecho = self.get_clock().now()

        if alvo is not None:
            self.frames_vistos += 1
            # Só começa a alinhar depois de ver a tag por alguns frames seguidos:
            # um único frame na borda da imagem não é confiável.
            if self.frames_vistos >= self.get_parameter('frames_para_avistar').value:
                self.log_deteccao(f'(listener) Tag {self.alvo_id} avistada! Iniciando alinhamento.', no_topico=(self.tipo_alvo == 'cubo'))
                #self.log_deteccao(f'(listener) Cubo da cor {alvo.get("cor_cubo", "não identificada")}', no_topico=(self.tipo_alvo == 'cubo'))
                self.frames_estaveis = 0
                self.frames_perdidos = 0
                self.resetar_filtro()
                self.resetar_rampa(vx=-float(self.get_parameter('search_vy').value) * self.sentido_busca)
                self.fase = Fase.ALINHANDO
                self.passo_alinhamento(alvo)
                return
        else:
            self.frames_vistos = 0

        # O 1º trecho sai do ponto de partida e vai até uma ponta (T). Os seguintes precisam
        # do DOBRO do tempo: T para voltar ao ponto de partida + T para varrer o outro lado.
        # Com o mesmo T em todos, o 2º trecho só voltava ao início e o lado oposto ao
        # sentido inicial nunca era varrido.
        tempo_trecho = float(self.get_parameter('search_sweep_time').value)
        if self.trechos_feitos > 0:
            tempo_trecho *= 2.0
        max_trechos = self.get_parameter('search_max_legs').value
        decorrido = (self.get_clock().now() - self.inicio_trecho).nanoseconds / 1e9

        # Se a tag está aparecendo agora (frames_vistos > 0, ainda confirmando), não encerra
        # o trecho neste frame: espera a confirmação (no máximo frames_para_avistar frames).
        if decorrido >= tempo_trecho and self.frames_vistos == 0:
            self.trechos_feitos += 1
            if max_trechos > 0 and self.trechos_feitos >= max_trechos:
                # Busca esgotada: em vez de só parar (o que travaria o plano), responde como
                # no 'impossivel': eco do comando em /topico_garra_status, alinhamento fingido.
                self.get_logger().warning(
                    f'(listener) [BUSCA ESGOTADA] Tag {self.alvo_id} não encontrada após {self.trechos_feitos} trechos. '
                    f'Vou responder como se o alvo tivesse sido pego, para o plano continuar.')
                self.concluir_alinhamento(fingido=True)
                self.get_logger().warning(
                    f'(listener) [BUSCA ESGOTADA] Eco enviado (alinhamento FINGIDO, nada foi pego): {self.comando_atual}. '
                    f'A garra segue para o próximo passo.')
                return
            self.sentido_busca *= -1
            self.inicio_trecho = self.get_clock().now()
            self.get_logger().info('(listener) Tag não encontrada neste trecho; invertendo o sentido da busca.')

        twist = Twist()
        twist.linear.y = float(self.get_parameter('search_vy').value) * self.sentido_busca
        self.publicar_cmd(twist)

    def passo_alinhamento(self, alvo):
        """Fase 2: alinhamento fino; se a tag some por muitos frames, volta a buscar."""
        if alvo is None:
            self.frames_estaveis = 0
            self.frames_perdidos += 1

            # Desacelera aos poucos (também reduz o borrão e ajuda a reencontrar a tag)
            DV_FREIO = 0.04
            self.vx_ant = math.copysign(max(abs(self.vx_ant) - DV_FREIO, 0.0), self.vx_ant)
            self.vy_ant = math.copysign(max(abs(self.vy_ant) - DV_FREIO, 0.0), self.vy_ant)
            twist = Twist()
            twist.linear.x = float(self.vy_ant)
            twist.linear.y = float(-self.vx_ant)
            self.publicar_cmd(twist)
            # Sem tag não há erro medido: o integral fica congelado, e o tempo sem tag
            # não conta como dt quando ela voltar
            self.t_pi_ant = None

            if self.frames_perdidos >= self.get_parameter('frames_perdida_limite').value:
                self.get_logger().warning(f'(listener) Tag {self.alvo_id} perdida durante o alinhamento; voltando a buscar.')
                self.sentido_busca = self.sentido_para(self.ultimo_erro_lateral < 0)
                self.inicio_trecho = self.get_clock().now()
                self.trechos_feitos = 0
                self.frames_vistos = 0
                self.fase = Fase.BUSCANDO
            return

        self.frames_perdidos = 0

        cam_x, _, cam_z = alvo["pose_3d"]
        alvo_x, _ = self.ALVOS[self.tipo_alvo][self.altura_mesa_cm]
        self.ultimo_erro_lateral = cam_x - alvo_x

        # Contêiner com o topo cortado = ainda longe: só avança em linha reta
        if self.tipo_alvo == 'container' and alvo["bbox"][1] <= Y_TOPO_CORTADO:
            self.resetar_filtro()
            self.resetar_rampa()
            self.frames_estaveis = 0
            twist = Twist()
            twist.linear.x = float(self.get_parameter('velocidade_cega_container').value)
            self.publicar_cmd(twist)
            self.get_logger().info(
                '(listener) [contêiner] topo fora da imagem: avançando em linha reta',
                throttle_duration_sec=1.0)
            return

        # Suaviza X e Z para QUALQUER alvo (antes era só o contêiner)
        if self.tipo_alvo == 'container':
            a = float(self.get_parameter('suavizacao_container').value)
        else:
            a = float(self.get_parameter('suavizacao_tag').value)

        if self.x_filt is None:
            self.x_filt, self.z_filt = cam_x, cam_z
        else:
            self.x_filt = a * cam_x + (1 - a) * self.x_filt
            self.z_filt = a * cam_z + (1 - a) * self.z_filt
        cam_x, cam_z = self.x_filt, self.z_filt

        # Log de calibração (só contêiner, como antes)
        if self.tipo_alvo == 'container':
            bx, by, bw, bh = alvo["bbox"]
            self.get_logger().info(
                f'(listener) [calibração contêiner] X={cam_x:.4f}  Z={cam_z:.4f}  h_px={alvo.get("h_px", 0)}'
                f'  y_topo={by}  y_base={by + bh}  x_esq={bx}  x_dir={bx + bw}',
                throttle_duration_sec=0.5)

        v_frente, v_lado = self.calcular_velocidade_aproximacao(
            cam_x, cam_z, self.tipo_alvo, self.altura_mesa_cm)

        twist = Twist()
        twist.linear.x = float(v_frente)
        twist.linear.y = float(v_lado)
        self.publicar_cmd(twist)

        if v_frente == 0.0 and v_lado == 0.0:
            self.frames_estaveis += 1
            if self.frames_estaveis >= self.get_parameter('stable_frames').value:
                self.concluir_alinhamento()
        else:
            self.frames_estaveis = 0

    # Calibração: média de X e Z de cada tag à vista
    def log_calibracao(self, results):
        if not self.get_parameter('log_tags').value:
            self.calib_amostras = {}
            return
        n = int(self.get_parameter('calib_frames').value)
        for r in results:
            if r["id"] >= 0:
                x, _, z = r["pose_3d"]
                self.calib_amostras.setdefault(r["id"], []).append((x, z))
        for tag_id, amostras in list(self.calib_amostras.items()):
            if len(amostras) < n:
                continue
            xs = [a[0] for a in amostras]
            zs = [a[1] for a in amostras]
            mx, mz = sum(xs) / n, sum(zs) / n
            dx = math.sqrt(sum((v - mx) ** 2 for v in xs) / n)
            dz = math.sqrt(sum((v - mz) ** 2 for v in zs) / n)
            self.get_logger().info(
                f'(listener) [calibração] tag {tag_id}: X={mx:.4f} (±{dx:.4f})  Z={mz:.4f} (±{dz:.4f})'
                f'  média de {n} frames  ->  ({mx:.3f}, {mz:.3f})')
            self.calib_amostras[tag_id] = []

    # Callback da Câmera
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_frame = frame.copy()

            color_objects = self.color_detector.detect(frame)
            tags = self.tag_detector.detect(frame)
            agora = self.get_clock().now().nanoseconds / 1e9
            results = self.classifier.classify(tags, color_objects, agora)

            # Registra a ordem das tags. Isolado num try: se a memória der erro, a busca,
            # o alinhamento e o eco fingido continuam rodando neste frame.
            try:
                self.registrar_tags(results)
            except Exception as e:
                self.get_logger().error(f'(listener) Erro ao registrar a ordem das tags: {e}')

            self.log_calibracao(results)

            # Zonas (101..107) à vista neste frame: decidem o lado da busca por uma zona
            self.zonas_visiveis = [r["id"] for r in results
                                   if ZONA_TAG_BASE < r["id"] <= ZONA_TAG_BASE + ZONA_N_MAX]

            alvo_encontrado = None

            for item in results:
                x, y, w, h = item["bbox"]
                cor_bgr = (0, 0, 255) if item["color"] == "vermelho" else (255, 0, 0) if item["color"] == "azul" else (255, 255, 255)
                cv2.rectangle(display_frame, (x, y), (x + w, y + h), cor_bgr, 2)
                label = f"{item['waypoint'].upper()} | ID:{item['id']} | {item['color']}"
                cv2.putText(display_frame, label, (x, max(y - 10, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cor_bgr, 2)

                if item["waypoint"] == "container":
                    px, _, pz = item["pose_3d"]
                    cv2.putText(display_frame, f"X={px:.3f} Z={pz:.3f}", (x, y + h + 18),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, cor_bgr, 2)

                if self.alvo_id is not None and item["id"] == self.alvo_id:
                    if self.tipo_alvo == 'container':
                        # Mais de um da mesma cor: pega o maior (mais próximo)
                        if alvo_encontrado is None or item["h_px"] > alvo_encontrado["h_px"]:
                            alvo_encontrado = item
                    elif alvo_encontrado is None or item["waypoint"] == self.tipo_alvo:
                        alvo_encontrado = item

            # Na busca, contêiner cortado nas bordas laterais da imagem ainda não vale como avistado
            alvo_busca = alvo_encontrado
            alvo_na_borda = False
            if (self.fase == Fase.BUSCANDO and self.tipo_alvo == 'container'
                    and alvo_encontrado is not None):
                bx, _, bw, _ = alvo_encontrado["bbox"]
                if bx <= BORDA_LATERAL_PX or bx + bw >= frame.shape[1] - BORDA_LATERAL_PX:
                    alvo_busca = None
                    alvo_na_borda = True

            if self.fase == Fase.BUSCANDO and self.tipo_alvo == 'container':
                for it in results:
                    if it["waypoint"] == "container":
                        self.get_logger().info(
                            f'(listener) [debug contêiner] id={it["id"]} (comando: {self.alvo_id}) '
                            f'h_px={it["h_px"]} bbox={it["bbox"]} na_borda={alvo_na_borda} '
                            f'frames_vistos={self.frames_vistos}',
                            throttle_duration_sec=0.3)
            if self.fase == Fase.BUSCANDO:
                self.passo_busca(alvo_busca)
            elif self.fase == Fase.ALINHANDO:
                self.passo_alinhamento(alvo_encontrado)
            elif self.fase == Fase.INSPECIONANDO:
                self.passo_inspecao(results)
            elif self.fase == Fase.GUARDANDO:
                self.passo_guardar(results)
            # Fase.IDLE: não publica nada em /cmd_vel

            texto = f"FASE: {self.fase.name}"
            if self.fase != Fase.IDLE:
                if alvo_na_borda:
                    texto += " | ALVO NA BORDA"
                else:
                    texto += " | ALVO VISTO" if alvo_encontrado is not None else " | ALVO NAO VISTO"
            cv2.putText(display_frame, texto, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Visão do Robô", display_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'(listener) Erro no processamento da imagem: {e}')


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
