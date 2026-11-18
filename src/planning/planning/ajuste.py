import math

import rclpy

from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class AproximarRotacionar(Node):

    def __init__(self):
        super().__init__('aproximar_rotacionar')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_ajuste = self.create_publisher(String, '/topico_garra_status', 10)

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.callback, qos_profile_sensor_data
        )
        self.sub_garra = self.create_subscription(
            String, '/topico_garra', self.garra_callback, 10
        )

        self.distancia_alvo = 0.28
        self.tolerancia = 0.02
        self.velocidade_frente = 0.20
        self.velocidade_ajuste = 0.10

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
            return

        self.comando_atual = msg.data
        self.distancia_atual = None
        self.distancia_esquerda = None
        self.distancia_direita = None
        self.erro_lateral = None
        self.ultima_leitura = None
        self.ultimo_scan_recebido = None
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
    # LEITURA DO LIDAR
    # =====================================

    def callback(self, msg):
        agora = self.get_clock().now()
        self.ultimo_scan_recebido = agora

        self.distancia_atual = None
        self.distancia_esquerda = None
        self.distancia_direita = None
        self.erro_lateral = None
        self.lados_validos = 0

        if not msg.ranges or msg.angle_increment == 0.0:
            return

        quantidade = len(msg.ranges)
        abertura = abs(msg.angle_increment) * quantidade
        if abs(abertura - 2.0 * math.pi) > math.radians(5.0):
            return

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

        # Mantém as laterais mesmo quando a frente é inf
        if valida(frente):
            self.distancia_atual = frente
        if valida(esquerda):
            self.distancia_esquerda = esquerda
        if valida(direita):
            self.distancia_direita = direita

        # Com a frente válida, preserva o filtro de discrepância original
        if self.distancia_atual is not None:
            esperada = self.distancia_atual / math.cos(
                math.radians(self.angulo_lateral)
            )
            if (self.distancia_esquerda is not None and
                    self.distancia_esquerda - esperada > self.tolerancia_discrepancia):
                self.distancia_esquerda = None
            if (self.distancia_direita is not None and
                    self.distancia_direita - esperada > self.tolerancia_discrepancia):
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

        # Guarda a direção para uma eventual recuperação futura.
        if self.erro_lateral is not None and abs(self.erro_lateral) > self.tolerancia_lateral:
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
                self.erro_ajuste('Leitura frontal não recuperada')
            return

        # Direção baseada nas duas laterais, mesmo se a frente estiver inf.
        sentido = None
        if (self.distancia_esquerda is not None and
                self.distancia_direita is not None and
                abs(self.distancia_esquerda - self.distancia_direita) > self.tolerancia_lateral):
            sentido = math.copysign(
                1.0,
                self.sinal_rotacao * (self.distancia_esquerda - self.distancia_direita)
            )
        elif self.ultimo_sentido_conhecido is not None:
            sentido = self.ultimo_sentido_conhecido

        # Não girar sem scans recentes ou sem referência confiável de direção.
        if sentido is None:
            if tempo_sem_frente >= self.timeout:
                self.erro_ajuste('Sem direção confiável para recuperar leitura frontal')
            return

        # Exige leituras laterais atuais, dentro de um intervalo conservador.
        laterais = [d for d in (self.distancia_esquerda, self.distancia_direita)
                    if d is not None]
        if not laterais or not all(
            self.distancia_lateral_minima_recuperacao <= d <=
            self.distancia_lateral_maxima_recuperacao for d in laterais
        ):
            if tempo_sem_frente >= self.timeout:
                self.erro_ajuste('Sem distância lateral segura para recuperação')
            return

        self.sentido_recuperacao = sentido
        self.tentativas_recuperacao += 1
        self.inicio_giro_recuperacao = agora
        self.estado = 'RECUPERANDO'
        self.get_logger().warn('Iniciando pequeno giro para recuperar leitura frontal.')

    # =====================================
    # CONTROLE PRINCIPAL
    # =====================================

    def controlar(self):
        if self.estado in ('AGUARDANDO', 'ERRO'):
            return

        agora = self.get_clock().now()
        cmd = Twist()
        tempo_ajuste = (agora - self.inicio_ajuste).nanoseconds / 1e9
        if tempo_ajuste > self.timeout_ajuste:
            self.erro_ajuste('Tempo máximo de ajuste excedido')
            return

        # Sem mensagens /scan, nunca tentar girar às cegas.
        if (self.ultimo_scan_recebido is None or
                (agora - self.ultimo_scan_recebido).nanoseconds / 1e9 > self.timeout):
            self.parar()
            if tempo_ajuste > self.timeout:
                self.erro_ajuste('Timeout: sem mensagens do LiDAR')
            return

        # A recuperação é exclusivamente rotacional e por tempo limitado.
        if self.estado == 'RECUPERANDO':
            decorrido = (agora - self.inicio_giro_recuperacao).nanoseconds / 1e9
            if decorrido >= self.duracao_giro_recuperacao:
                self.parar()
                self.estado = 'ESPERANDO_FRENTE'
                self.inicio_espera_pos_giro = agora
                return

            laterais = [d for d in (self.distancia_esquerda, self.distancia_direita)
                        if d is not None]
            if (not laterais or
                    any(d < self.distancia_lateral_minima_recuperacao for d in laterais)):
                self.erro_ajuste('Perda de referência lateral durante giro')
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
            if (agora - self.inicio_espera_pos_giro).nanoseconds / 1e9 >= self.espera_recuperacao:
                self.erro_ajuste('Frente continua inválida após giro de recuperação')
            return

        if self.distancia_atual is None:
            self.controlar_sem_frente(agora)
            return

        self.inicio_sem_frente = None
        distancia = self.distancia_atual

        if distancia < self.distancia_minima_segura:
            self.erro_ajuste('Mesa próxima demais do LiDAR')
            return

        if self.erro_lateral is None:
            self.parar()
            self.verificacoes = 0
            return

        erro_lateral = self.erro_lateral

        # ETAPA 1: APROXIMAR
        if self.estado == 'APROXIMANDO':
            if distancia > self.distancia_alvo + self.tolerancia:
                cmd.linear.x = self.velocidade_frente
            elif distancia < self.distancia_alvo - self.tolerancia:
                cmd.linear.x = -self.velocidade_ajuste
            else:
                self.parar()
                self.estado = 'ALINHANDO'
                self.get_logger().info('Distância alcançada. Iniciando alinhamento.')
                return

        # ETAPA 2: ALINHAR
        elif self.estado == 'ALINHANDO':
            if abs(erro_lateral) <= self.tolerancia_lateral:
                self.parar()
                self.estado = 'VERIFICANDO'
                self.verificacoes = 0
                self.get_logger().info('Alinhamento alcançado. Verificando.')
                return

            velocidade = self.kp_rotacao * erro_lateral
            velocidade = max(-self.velocidade_angular_max,
                             min(self.velocidade_angular_max, velocidade))
            if abs(velocidade) < self.velocidade_angular_min:
                velocidade = math.copysign(self.velocidade_angular_min, velocidade)
            cmd.angular.z = self.sinal_rotacao * velocidade

        # ETAPA 3: VERIFICAR
        elif self.estado == 'VERIFICANDO':
            distancia_correta = abs(distancia - self.distancia_alvo) <= self.tolerancia
            alinhamento_correto = abs(erro_lateral) <= self.tolerancia_lateral
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
            f'Estado: {self.estado} | Frente: {self.distancia_atual} | '
            f'Esquerda: {self.distancia_esquerda} | Direita: {self.distancia_direita} | '
            f'Erro: {erro_lateral:.3f} | Vx: {cmd.linear.x:.2f} | Wz: {cmd.angular.z:.2f}'
        )

    # =====================================
    # FINALIZAÇÃO
    # =====================================

    def finalizar_ajuste(self):
        self.parar()
        self.estado = 'AGUARDANDO'
        msg = String()
        msg.data = self.comando_atual
        self.publisher_ajuste.publish(msg)
        self.get_logger().info('Ajuste concluído.')
        self.comando_atual = None
        self.inicio_ajuste = None

    # =====================================
    # ERRO
    # =====================================

    def erro_ajuste(self, motivo):
        self.parar()
        self.estado = 'ERRO'
        self.get_logger().error(f'Ajuste interrompido: {motivo}')
        msg = String()
        msg.data = 'erro no ajuste'
        self.publisher_ajuste.publish(msg)

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