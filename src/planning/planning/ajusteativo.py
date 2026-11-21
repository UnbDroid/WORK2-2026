import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class AproximarRotacionar(Node):
    def __init__(self):
        super().__init__('aproximar_rotacionar')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_ajuste = self.create_publisher(String, '/topico_garra_status', 10)
        self.publisher_garra = self.create_publisher(String, '/topico_garra', 10)

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.callback, qos_profile_sensor_data
        )
        self.sub_garra = self.create_subscription(
            String, '/topico_garra', self.garra_callback, 10
        )
        self.sub_garra_status = self.create_subscription(
            String, '/topico_garra_status', self.garra_status_callback, 10
        )

        # Distância de segurança das paredes laterais
        self.distancia_limite_parede = 0.32
        self.distancia_parede_90 = None
        self.distancia_parede_270 = None
        self.ultimo_scan_valido = None

        # Aproximação
        self.distancia_alvo = 0.32
        self.tolerancia = 0.02
        self.velocidade_frente = 0.20
        self.velocidade_ajuste = 0.10

        # Alinhamento
        self.angulo_lateral = 30.0
        self.tolerancia_lateral = 0.01
        self.tolerancia_discrepancia = 0.15
        self.kp_rotacao = 2.0
        self.velocidade_angular_max = 0.7
        self.velocidade_angular_min = 0.3
        self.sinal_rotacao = -1.0

        # LiDAR
        self.distancia_atual = None
        self.distancia_esquerda = None
        self.distancia_direita = None
        self.erro_lateral = None
        self.lados_validos = 0
        self.ultima_leitura = None
        self.ultimo_scan_recebido = None
        self.numero_scan = 0
        self.ultimo_scan_verificado = -1

        # Estados
        self.estado = 'AGUARDANDO'
        self.comando_atual = None
        self.verificacoes = 0
        self.verificacoes_necessarias = 4
        self.inicio_ajuste = None

        # Segurança
        self.timeout = 2.0
        self.timeout_ajuste = 30.0
        self.distancia_minima_segura = 0.12

        # Recuperação quando a leitura frontal é inf
        self.espera_recuperacao = 0.5
        self.duracao_giro_recuperacao = 0.20
        self.velocidade_giro_recuperacao = 0.30
        self.tentativas_maximas = 1
        self.tentativas_recuperacao = 0
        self.inicio_sem_frente = None
        self.inicio_giro_recuperacao = None
        self.inicio_espera_pos_giro = None
        self.sentido_recuperacao = None
        self.ultimo_sentido_conhecido = None
        self.distancia_lateral_minima_recuperacao = 0.20
        self.distancia_lateral_maxima_recuperacao = 0.90

        self.timer = self.create_timer(0.05, self.controlar)
        self.get_logger().info('Nó iniciado. Aguardando comando ajustar pelo tópico.')

    # =====================================
    # ATIVAÇÃO PELA GARRA
    # =====================================
    def garra_callback(self, msg):
        if msg.data != 'ajustar' and not msg.data.startswith('ajustar:'):
            return

        if self.estado != 'AGUARDANDO':
            self.get_logger().warn(
                f'Comando ajustar ignorado: estado atual {self.estado}'
            )
            return

        self.comando_atual = msg.data
        self.distancia_atual = None
        self.distancia_esquerda = None
        self.distancia_direita = None
        self.erro_lateral = None
        self.ultima_leitura = None
        self.ultimo_scan_recebido = None
        self.ultimo_scan_valido = None
        self.distancia_parede_90 = None
        self.distancia_parede_270 = None
        self.lados_validos = 0
        self.verificacoes = 0
        self.ultimo_scan_verificado = -1
        self.tentativas_recuperacao = 0
        self.inicio_sem_frente = None
        self.inicio_giro_recuperacao = None
        self.inicio_espera_pos_giro = None
        self.sentido_recuperacao = None
        self.ultimo_sentido_conhecido = None

        self.inicio_ajuste = self.get_clock().now()
        self.estado = 'APROXIMANDO'
        self.get_logger().info('Comando recebido. Iniciando ajuste pelo LiDAR.')

    # =====================================
    # STATUS DA GARRA
    # =====================================
    def garra_status_callback(self, msg):
        if self.estado != 'MONITORANDO':
            return

        if msg.data.startswith('transladar:'):
            self.get_logger().info('Transladar recebido. Encerrando monitoramento.')
            self.estado = 'AGUARDANDO'
            self.comando_atual = None
            self.inicio_ajuste = None

    # =====================================
    # LEITURA DO LIDAR
    # =====================================
    def callback(self, msg):
        agora = self.get_clock().now()
        self.ultimo_scan_recebido = agora

        self.distancia_atual = None
        self.distancia_esquerda = None
        self.distancia_direita = None
        self.distancia_parede_90 = None
        self.distancia_parede_270 = None
        self.erro_lateral = None
        self.lados_validos = 0

        if not msg.ranges or msg.angle_increment == 0.0:
            return

        quantidade = len(msg.ranges)
        abertura = abs(msg.angle_increment) * quantidade
        if abs(abertura - 2.0 * math.pi) > math.radians(5.0):
            return

        self.ultimo_scan_valido = agora

        # LiDAR invertido: último índice aponta para frente
        indice_frente = quantidade - 1
        deslocamento = round(
            math.radians(self.angulo_lateral) / abs(msg.angle_increment)
        )
        indice_esquerda = (indice_frente - deslocamento) % quantidade
        indice_direita = (indice_frente + deslocamento) % quantidade

        frente = msg.ranges[indice_frente]
        esquerda = msg.ranges[indice_esquerda]
        direita = msg.ranges[indice_direita]

        def valida(d):
            return math.isfinite(d) and msg.range_min <= d <= msg.range_max

        # Leitura das paredes a 90° e 270°
        deslocamento_90 = round(math.radians(90.0) / abs(msg.angle_increment))
        indice_90 = (indice_frente - deslocamento_90) % quantidade
        indice_270 = (indice_frente + deslocamento_90) % quantidade

        leitura_90 = msg.ranges[indice_90]
        leitura_270 = msg.ranges[indice_270]
        if valida(leitura_90):
            self.distancia_parede_90 = leitura_90
        if valida(leitura_270):
            self.distancia_parede_270 = leitura_270

        if valida(frente):
            self.distancia_atual = frente
        if valida(esquerda):
            self.distancia_esquerda = esquerda
        if valida(direita):
            self.distancia_direita = direita

        # Filtro de discrepância original
        if self.distancia_atual is not None:
            esperada = self.distancia_atual / math.cos(
                math.radians(self.angulo_lateral)
            )
            if (
                self.distancia_esquerda is not None
                and self.distancia_esquerda - esperada > self.tolerancia_discrepancia
            ):
                self.distancia_esquerda = None
            if (
                self.distancia_direita is not None
                and self.distancia_direita - esperada > self.tolerancia_discrepancia
            ):
                self.distancia_direita = None

        self.lados_validos = int(self.distancia_esquerda is not None) + int(
            self.distancia_direita is not None
        )
        if self.lados_validos == 2:
            self.erro_lateral = self.distancia_esquerda - self.distancia_direita
        elif self.distancia_atual is not None:
            esperada = self.distancia_atual / math.cos(
                math.radians(self.angulo_lateral)
            )
            if self.distancia_esquerda is not None:
                self.erro_lateral = 2.0 * (self.distancia_esquerda - esperada)
            elif self.distancia_direita is not None:
                self.erro_lateral = 2.0 * (esperada - self.distancia_direita)

        if (
            self.erro_lateral is not None
            and abs(self.erro_lateral) > self.tolerancia_lateral
        ):
            self.ultimo_sentido_conhecido = math.copysign(
                1.0, self.sinal_rotacao * self.erro_lateral
            )

        if self.distancia_atual is not None:
            self.ultima_leitura = agora
        self.numero_scan += 1

    # =====================================
    # RECUPERAÇÃO DA LEITURA FRONTAL
    # =====================================
    def controlar_sem_frente(self, agora):
        self.parar()
        self.verificacoes = 0
        if self.inicio_sem_frente is None:
            self.inicio_sem_frente = agora
            self.get_logger().warn('Frente inválida: aguardando recuperação.')

        tempo_sem_frente = (agora - self.inicio_sem_frente).nanoseconds / 1e9
        if tempo_sem_frente < self.espera_recuperacao:
            return

        if self.tentativas_recuperacao >= self.tentativas_maximas:
            if tempo_sem_frente >= self.timeout:
                self.abandonar_ajuste('Leitura frontal não recuperada')
            return

        sentido = None
        if (
            self.distancia_esquerda is not None
            and self.distancia_direita is not None
            and abs(self.distancia_esquerda - self.distancia_direita)
            > self.tolerancia_lateral
        ):
            sentido = math.copysign(
                1.0,
                self.sinal_rotacao * (
                    self.distancia_esquerda - self.distancia_direita
                ),
            )
        elif self.ultimo_sentido_conhecido is not None:
            sentido = self.ultimo_sentido_conhecido

        if sentido is None:
            if tempo_sem_frente >= self.timeout:
                self.abandonar_ajuste(
                    'Sem direção confiável para recuperar leitura frontal'
                )
            return

        laterais = [
            d for d in (self.distancia_esquerda, self.distancia_direita)
            if d is not None
        ]
        if not laterais or not all(
            self.distancia_lateral_minima_recuperacao
            <= d <= self.distancia_lateral_maxima_recuperacao
            for d in laterais
        ):
            if tempo_sem_frente >= self.timeout:
                self.abandonar_ajuste(
                    'Sem distância lateral segura para recuperação'
                )
            return

        self.sentido_recuperacao = sentido
        self.tentativas_recuperacao += 1
        self.inicio_giro_recuperacao = agora
        self.estado = 'RECUPERANDO'
        self.get_logger().warn(
            'Iniciando pequeno giro para recuperar leitura frontal.'
        )

    # =====================================
    # MONITORAMENTO DAS PAREDES
    # =====================================
    def monitorar_paredes(self, agora):
        if (
            self.ultimo_scan_valido is None
            or (agora - self.ultimo_scan_valido).nanoseconds / 1e9 > self.timeout
        ):
            self.get_logger().warn(
                'LiDAR sem scans válidos. Encerrando monitoramento lateral; '
                'não é possível garantir espaço livre.'
            )
            self.estado = 'AGUARDANDO'
            self.comando_atual = None
            self.inicio_ajuste = None
            return

        parede_90_perto = (
            self.distancia_parede_90 is not None
            and self.distancia_parede_90 < self.distancia_limite_parede
        )
        parede_270_perto = (
            self.distancia_parede_270 is not None
            and self.distancia_parede_270 < self.distancia_limite_parede
        )
        if parede_90_perto or parede_270_perto:
            self.get_logger().warn(
                f'Parede lateral a menos de '
                f'{self.distancia_limite_parede * 100:.0f} cm! '
                'Publicando impossivel em /topico_garra.'
            )
            self.publisher_garra.publish(String(data='impossivel'))
            self.estado = 'AGUARDANDO'
            self.comando_atual = None
            self.inicio_ajuste = None

    # =====================================
    # CONTROLE PRINCIPAL
    # =====================================
    def controlar(self):
        if self.estado in ('AGUARDANDO', 'ERRO'):
            return

        agora = self.get_clock().now()
        cmd = Twist()

        if self.estado == 'MONITORANDO':
            self.monitorar_paredes(agora)
            return

        tempo_ajuste = (agora - self.inicio_ajuste).nanoseconds / 1e9
        if tempo_ajuste > self.timeout_ajuste:
            self.abandonar_ajuste('Tempo máximo de ajuste excedido')
            return

        if (
            self.ultimo_scan_valido is None
            or (agora - self.ultimo_scan_valido).nanoseconds / 1e9 > self.timeout
        ):
            self.parar()
            if tempo_ajuste > self.timeout:
                self.abandonar_ajuste('Timeout: sem scans válidos do LiDAR')
            return

        if self.estado == 'RECUPERANDO':
            decorrido = (agora - self.inicio_giro_recuperacao).nanoseconds / 1e9
            if decorrido >= self.duracao_giro_recuperacao:
                self.parar()
                self.estado = 'ESPERANDO_FRENTE'
                self.inicio_espera_pos_giro = agora
                return

            laterais = [
                d for d in (self.distancia_esquerda, self.distancia_direita)
                if d is not None
            ]
            if not laterais or any(
                d < self.distancia_lateral_minima_recuperacao for d in laterais
            ):
                self.abandonar_ajuste('Perda de referência lateral durante giro')
                return

            cmd.angular.z = (
                self.sentido_recuperacao * self.velocidade_giro_recuperacao
            )
            self.publisher.publish(cmd)
            return

        if self.estado == 'ESPERANDO_FRENTE':
            self.parar()
            if self.distancia_atual is not None:
                self.inicio_sem_frente = None
                self.estado = 'APROXIMANDO'
                self.get_logger().info('Leitura frontal recuperada.')
                return
            if (
                (agora - self.inicio_espera_pos_giro).nanoseconds / 1e9
                >= self.espera_recuperacao
            ):
                self.abandonar_ajuste(
                    'Frente continua inválida após giro de recuperação'
                )
            return

        if self.distancia_atual is None:
            self.controlar_sem_frente(agora)
            return

        self.inicio_sem_frente = None
        distancia = self.distancia_atual

        # Risco de colisão: NÃO libera a missão automaticamente
        if distancia < self.distancia_minima_segura:
            self.erro_ajuste('Mesa próxima demais do LiDAR')
            return

        if self.erro_lateral is None:
            self.parar()
            self.verificacoes = 0
            return

        erro_lateral = self.erro_lateral

        if self.estado == 'APROXIMANDO':
            if distancia > self.distancia_alvo + self.tolerancia:
                cmd.linear.x = self.velocidade_frente
            elif distancia < self.distancia_alvo - self.tolerancia:
                cmd.linear.x = -self.velocidade_ajuste
            else:
                self.parar()
                self.estado = 'ALINHANDO'
                self.get_logger().info(
                    'Distância alcançada. Iniciando alinhamento.'
                )
                return

        elif self.estado == 'ALINHANDO':
            if abs(erro_lateral) <= self.tolerancia_lateral:
                self.parar()
                self.estado = 'VERIFICANDO'
                self.verificacoes = 0
                self.get_logger().info('Alinhamento alcançado. Verificando.')
                return

            velocidade = self.kp_rotacao * erro_lateral
            velocidade = max(
                -self.velocidade_angular_max,
                min(self.velocidade_angular_max, velocidade),
            )
            if abs(velocidade) < self.velocidade_angular_min:
                velocidade = math.copysign(
                    self.velocidade_angular_min, velocidade
                )
            cmd.angular.z = self.sinal_rotacao * velocidade

        elif self.estado == 'VERIFICANDO':
            distancia_correta = (
                abs(distancia - self.distancia_alvo) <= self.tolerancia
            )
            alinhamento_correto = (
                abs(erro_lateral) <= self.tolerancia_lateral
            )
            if not alinhamento_correto:
                self.verificacoes = 0
                self.estado = 'ALINHANDO'
                self.parar()
                return
            if not distancia_correta:
                self.verificacoes = 0
                self.estado = 'APROXIMANDO'
                self.parar()
                return
            if self.numero_scan != self.ultimo_scan_verificado:
                self.verificacoes += 1
                self.ultimo_scan_verificado = self.numero_scan
            if self.verificacoes >= self.verificacoes_necessarias:
                self.finalizar_ajuste()
            return

        self.publisher.publish(cmd)
        self.get_logger().debug(
            f'Estado: {self.estado} | '
            f'Frente: {self.distancia_atual} | '
            f'Esquerda: {self.distancia_esquerda} | '
            f'Direita: {self.distancia_direita} | '
            f'Erro: {erro_lateral:.3f} | '
            f'Vx: {cmd.linear.x:.2f} | Wz: {cmd.angular.z:.2f}'
        )

    # =====================================
    # FINALIZAÇÃO DO AJUSTE
    # =====================================
    def finalizar_ajuste(self):
        self.parar()
        self.estado = 'MONITORANDO'
        msg = String()
        msg.data = self.comando_atual
        self.publisher_ajuste.publish(msg)
        self.get_logger().info(
            'Ajuste concluído. Confirmação enviada em /topico_garra_status. '
            'Monitorando paredes laterais.'
        )

    # =====================================
    # NOVO: ABANDONO POR FALHA RECUPERÁVEL
    # =====================================
    def abandonar_ajuste(self, motivo):
        self.parar()
        self.get_logger().warn(
            f'Ajuste abandonado: {motivo}. '
            'Liberando missão sem alinhamento completo.'
        )
        if self.comando_atual is not None:
            self.publisher_ajuste.publish(String(data=self.comando_atual))
        self.estado = 'AGUARDANDO'
        self.comando_atual = None
        self.inicio_ajuste = None

    # =====================================
    # ERRO DE SEGURANÇA
    # =====================================
    def erro_ajuste(self, motivo):
        self.parar()
        self.estado = 'ERRO'
        self.get_logger().error(f'Ajuste interrompido: {motivo}')
        self.publisher_ajuste.publish(String(data='erro no ajuste'))

    # =====================================
    # PARADA
    # =====================================
    def parar(self):
        if rclpy.ok():
            self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = AproximarRotacionar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Encerrando')
    finally:
        node.parar()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()