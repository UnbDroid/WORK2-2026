from collections import Counter
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
H_MIN_container = 200

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


# 5. NÓ ROS 2
class VisionNode(Node):

    ALVOS = {
        "cubo": {
            5.0:  (0.185,    0.770),
            10.0: (0.173, 0.695),
            15.0: (0.163, 0.629),
        },
        "zona": {
            5.0:  (0.0, 0.0),   # CALIBRAR
            10.0: (0.0, 0.0),   # CALIBRAR
            15.0: (0.0, 0.0),   # CALIBRAR
        },
        "container": {
            5.0:  (0.130, 0.626),   
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
        self.declare_parameter('search_vy', -0.2)           # VELOCIDADE (m/s) da translação na busca; o sinal define o sentido "normal"
        self.declare_parameter('search_sweep_time', 30.0)   # TEMPO (s) de cada trecho da busca antes de inverter
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

        # Mesa PP: zonas visíveis no último frame e estado do inspecionar
        self.zonas_visiveis = []
        self.zona_inspecionada = None   # id da tag da zona (101..107)
        self.nome_zona_inspecao = None  # nome recebido no comando (ex.: zone1), devolvido na resposta
        self.votos_inspecao = []
        self.inicio_inspecao = None

        # Memória: local -> {id da tag: posição lateral estimada da tag (m)}
        self.memoria = {}
        self.local_atual = None

        # Posição lateral estimada do robô (m, +y = esquerda), integrando a velocidade comandada.
        # Serve só para decidir o sentido da busca: guardar o x da câmera (como era antes)
        # fica obsoleto assim que o robô anda.
        self.y_est = 0.0
        self.vy_ultimo = 0.0
        self.t_ultimo_cmd = None

        self.cubo_identificado_enviado = False #mudei

        self.sub = self.create_subscription(Image, '/cam/image_raw', self.image_callback, 10)
        self.sub_garra = self.create_subscription(String, '/topico_garra', self.garra_callback, 10)
        self.pub_garra = self.create_publisher(String, '/topico_garra', 10) #mudei
        self.pub_status = self.create_publisher(String, '/topico_garra_status', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

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

    # Callback de comando (único canal de controle: /topico_garra)
    def garra_callback(self, msg):
        cmd = msg.data
        if cmd.startswith('inspecionar:'):
            self.iniciar_inspecao(cmd)
            return
        if cmd.startswith('impossivel'):
            if self.fase in (Fase.BUSCANDO, Fase.ALINHANDO):
                self.get_logger().warning(
                    f'[IMPOSSÍVEL] "{cmd}" recebido na fase {self.fase.name} (alvo {self.comando_atual}). '
                    f'Parei de buscar/alinhar e vou responder como se o alvo tivesse sido pego.')
                self.concluir_alinhamento()
                self.get_logger().warning(
                    f'[IMPOSSÍVEL] Eco enviado (alinhamento FINGIDO, nada foi pego): {self.comando_atual}. '
                    f'A garra segue para o próximo passo.')
            else:
                self.get_logger().info(
                    f'[IMPOSSÍVEL] "{cmd}" ignorado: robô na fase {self.fase.name}, não há alinhamento para encerrar.')
            return
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

        if tipo == 'container' and alvo not in container_IDS.values():
            self.get_logger().error(
                f'Contêiner inválido (use -1 = vermelho, -2 = azul): {cmd}')
            return

        if tipo != 'container' and alvo < 0:
            self.get_logger().error(f'ID negativo só é válido para contêiner: {cmd}')
            return

        if tipo == 'zona':
            # zoneN (N = 1..7) -> tag 100+N
            if not 1 <= alvo <= ZONA_N_MAX:
                self.get_logger().error(f'Zona inválida (use zone1..zone{ZONA_N_MAX}): {cmd}')
                return
            alvo = ZONA_TAG_BASE + alvo

        if altura_cm not in self.ALVOS[tipo]:
            self.get_logger().error(f'{tipo} a {altura_cm} cm sem calibração. Disponíveis: {list(self.ALVOS[tipo])}')
            return

        if self.ALVOS[tipo][altura_cm] == (0.0, 0.0):
            self.get_logger().error(f'{tipo} a {altura_cm} cm ainda não calibrado (alvo 0, 0). Comando ignorado.')
            return

        if local != self.local_atual:
            # Mudou de local: o referencial lateral e a memória antiga não valem mais
            self.local_atual = local
            self.memoria[local] = {}
            self.y_est = 0.0

        self.comando_atual = cmd
        self.alvo_id = alvo
        self.tipo_alvo = tipo
        self.altura_mesa_cm = altura_cm
        self.frames_estaveis = 0
        self.resetar_filtro()
        self.get_logger().info(f'Comando: {cmd} -> {tipo} id {alvo}, mesa a {altura_cm} cm, local {local}')

        self.iniciar_busca()

    # Publicação de velocidade + estimativa de posição lateral
    def publicar_cmd(self, twist):
        agora = self.get_clock().now()
        if self.t_ultimo_cmd is not None:
            dt = (agora - self.t_ultimo_cmd).nanoseconds / 1e9
            if dt < 0.5:  # ignora lacunas (nó ficou em IDLE sem publicar)
                self.y_est += self.vy_ultimo * dt
        self.t_ultimo_cmd = agora
        self.vy_ultimo = twist.linear.y
        self.pub_cmd_vel.publish(twist)

    def parar_robo(self):
        for _ in range(3):
            self.publicar_cmd(Twist())

    # Memória e escolha de sentido
    def registrar_tags(self, results):
        if self.fase == Fase.IDLE or self.local_atual is None:
            return
        mem = self.memoria.setdefault(self.local_atual, {})
        for item in results:
            if item["id"] >= 0:
                # Posição lateral da tag no mesmo eixo de y_est (+ = esquerda).
                # O x da câmera é + para a direita, por isso o sinal de menos.
                mem[item["id"]] = self.y_est - item["pose_3d"][0]

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
            f'Zona {self.alvo_id - ZONA_TAG_BASE}: vendo {sorted(visiveis)} -> '
            f'vou para a {"esquerda" if esquerda else "direita"} '
            f'(sentido {"normal" if sentido == 1 else "inverso"})')
        return sentido

    def escolher_sentido(self):
        if self.tipo_alvo == 'zona':
            sentido = self.sentido_pela_ordem_das_zonas()
            if sentido is not None:
                return sentido

        mem = self.memoria.get(self.local_atual, {})
        y_tag = mem.get(self.alvo_id)

        if y_tag is None:
            self.get_logger().info(f'Alvo {self.alvo_id}: sem registro -> sentido normal')
            return 1

        delta = y_tag - self.y_est
        if abs(delta) < self.get_parameter('dist_min_memoria').value:
            self.get_logger().info(f'Alvo {self.alvo_id}: registrado aqui perto (delta={delta:.2f} m) -> sentido normal')
            return 1

        esquerda = delta > 0
        sentido = self.sentido_para(esquerda)
        self.get_logger().info(
            f'Alvo {self.alvo_id}: registrado a {abs(delta):.2f} m à '
            f'{"esquerda" if esquerda else "direita"} -> sentido {"normal" if sentido == 1 else "inverso"}')
        return sentido

    def iniciar_busca(self):
        self.sentido_busca = self.escolher_sentido()
        self.inicio_trecho = self.get_clock().now()
        self.trechos_feitos = 0
        self.frames_vistos = 0
        self.fase = Fase.BUSCANDO

    def concluir_alinhamento(self):
        self.parar_robo()
        self.fase = Fase.IDLE
        self.resetar_filtro()

        if self.tipo_alvo == 'cubo':
            self.memoria.get(self.local_atual, {}).pop(self.alvo_id, None)

        eco = String()
        eco.data = self.comando_atual
        self.pub_status.publish(eco)
        self.get_logger().info(f'Alinhado. Eco enviado em /topico_garra_status: {eco.data}')

    # Inspecionar: qual cubo está embaixo da zona em frente?
    def iniciar_inspecao(self, cmd):
        partes = cmd.split(':')
        if len(partes) != 2:
            self.get_logger().error(f'Formato inesperado (esperado inspecionar:zoneN): {cmd}')
            return

        nome = partes[1].strip()
        n = self.nome_para_id(nome)
        if self.nome_para_tipo(nome) != 'zona' or n is None or not 1 <= n <= ZONA_N_MAX:
            self.get_logger().error(f'Inspeção inválida (use inspecionar:zone1..zone{ZONA_N_MAX}): {cmd}')
            return

        if self.fase != Fase.IDLE:
            self.get_logger().error(f'Robô ocupado (fase {self.fase.name}); inspeção ignorada: {cmd}')
            return

        self.zona_inspecionada = ZONA_TAG_BASE + n
        self.nome_zona_inspecao = nome
        self.votos_inspecao = []
        self.inicio_inspecao = self.get_clock().now()
        self.fase = Fase.INSPECIONANDO
        self.get_logger().info(f'Inspecionando {nome} (tag {self.zona_inspecionada})')

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
                f'Tag da zona {self.zona_inspecionada} não apareceu em {decorrido:.0f} s; inspeção cancelada, sem resposta.')
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
        self.get_logger().info(f'Inspeção concluída (votos {dict(votos)}). Resposta em /topico_garra_status: {eco.data}')
        self.fase = Fase.IDLE

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
                vx = 0.15 if vx > 0 else -0.15

        elif abs(erro_frontal) > tolerancia_z:
            vy = Kp_z * erro_frontal
            vx = 0
            vy = max(min(vy, 0.5), -0.5)
            if abs(vy) < 0.1:
                vy = 0.15 if vy > 0 else -0.15

        else:
            vx = 0
            vy = 0

        print(f'Velocidade final: vx={vx:.3f} e vy={vy:.3f}  A coordenada X atual: {delta_x:.3f}, o erro lateral atual é:{erro_lateral:.3f} , A coordenada Y atual: {delta_z:.3f}, O erro frontal atual é:{erro_frontal:.3f}')

        return vy, -vx

    # Fases do Comportamento
    def passo_busca(self, alvo):
        """Fase 1: translada de lado; inverte o sentido a cada trecho sem achar a tag."""
        if alvo is not None:
            self.frames_vistos += 1
            # Só começa a alinhar depois de ver a tag por alguns frames seguidos:
            # um único frame na borda da imagem não é confiável.
            if self.frames_vistos >= self.get_parameter('frames_para_avistar').value:
                self.get_logger().info(f'Tag {self.alvo_id} avistada! Iniciando alinhamento.')
                self.get_logger().info(f'Cubo da cor {alvo.get("cor_cubo", "não identificada")}')
                self.frames_estaveis = 0
                self.frames_perdidos = 0
                self.resetar_filtro()
                self.fase = Fase.ALINHANDO
                self.passo_alinhamento(alvo)
                return
        else:
            self.frames_vistos = 0

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
        self.publicar_cmd(twist)

    def passo_alinhamento(self, alvo):
        """Fase 2: alinhamento fino; se a tag some por muitos frames, volta a buscar."""
        if alvo is None:
            self.frames_estaveis = 0
            self.frames_perdidos += 1
            self.resetar_filtro()
            self.publicar_cmd(Twist())

            if self.frames_perdidos >= self.get_parameter('frames_perdida_limite').value:
                self.get_logger().warning(f'Tag {self.alvo_id} perdida durante o alinhamento; voltando a buscar.')
                # Volta a varrer em direção ao lado onde a tag estava (erro lateral > 0 = tag à direita)
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

        # Contêiner: suaviza X e Z (a massa de cor é mais ruidosa que uma tag)
        if self.tipo_alvo == 'container':
            # Topo cortado = ainda longe: sem correção lateral nem cálculo de Z, só avança
            if alvo["bbox"][1] <= Y_TOPO_CORTADO:
                self.resetar_filtro()
                self.frames_estaveis = 0
                twist = Twist()
                twist.linear.x = float(self.get_parameter('velocidade_cega_container').value)
                self.publicar_cmd(twist)
                self.get_logger().info(
                    '[contêiner] topo fora da imagem: avançando em linha reta',
                    throttle_duration_sec=1.0)
                return

            a = float(self.get_parameter('suavizacao_container').value)
            if self.x_filt is None:
                self.x_filt, self.z_filt = cam_x, cam_z
            else:
                self.x_filt = a * cam_x + (1 - a) * self.x_filt
                self.z_filt = a * cam_z + (1 - a) * self.z_filt
            cam_x, cam_z = self.x_filt, self.z_filt

            # Use estes valores para calibrar o ponto ideal em ALVOS["container"]
            bx, by, bw, bh = alvo["bbox"]
            self.get_logger().info(
                f'[calibração contêiner] X={cam_x:.4f}  Z={cam_z:.4f}  h_px={alvo.get("h_px", 0)}'
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

    # Callback da Câmera
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_frame = frame.copy()

            color_objects = self.color_detector.detect(frame)
            tags = self.tag_detector.detect(frame)
            agora = self.get_clock().now().nanoseconds / 1e9
            results = self.classifier.classify(tags, color_objects, agora)

            # Registra todas as tags na memória por local e ID
            self.registrar_tags(results)

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

            if self.fase == Fase.BUSCANDO:
                self.passo_busca(alvo_busca)
            elif self.fase == Fase.ALINHANDO:
                self.passo_alinhamento(alvo_encontrado)
            elif self.fase == Fase.INSPECIONANDO:
                self.passo_inspecao(results)
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
