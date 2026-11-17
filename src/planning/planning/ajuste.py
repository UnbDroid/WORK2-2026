import math

import rclpy

# from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class AproximarRotacionar(Node):

    def __init__(self):
        super().__init__('aproximar_rotacionar')

        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # self.publisher_ajuste = self.create_publisher(
        #     String, '/topico_garra_status', 10
        # )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            qos_profile_sensor_data
        )

        # self.sub_garra = self.create_subscription(
        #     String,
        #     '/topico_garra',
        #     self.garra_callback,
        #     10
        # )

        self.distancia_alvo = 0.20
        self.tolerancia = 0.02
        self.velocidade_frente = 0.10
        self.velocidade_ajuste = 0.05

        self.angulo_lateral = 30.0  # graus em relação à frente
        self.tolerancia_lateral = 0.01  # diferença máxima entre os lados, em metros
        self.kp_rotacao = 2.0
        self.velocidade_angular_max = 0.7
        self.velocidade_angular_min = 0.3
        self.sinal_rotacao = 1.0  # inverter para -1.0 se girar para o lado errado

        # LiDAR
        self.distancia_atual = None
        self.distancia_esquerda = None
        self.distancia_direita = None
        self.ultima_leitura = None

        # Estados
        self.estado = "AGUARDANDO"
        self.comando_atual = None
        self.verificacoes = 0
        self.verificacoes_necessarias = 4
        self.inicio_ajuste = None

        # Segurança
        self.timeout = 2.0
        self.timeout_ajuste = 30.0
        self.distancia_minima_segura = 0.12

        # Timer de controle: 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.controlar
        )

        # Teste sem garra: inicia o ajuste automaticamente
        self.inicio_ajuste = self.get_clock().now()
        self.estado = "APROXIMANDO"

        self.get_logger().info(
            'Nó iniciado. Iniciando aproximação e alinhamento pelo LiDAR.'
        )

    # =====================================
    # ATIVAÇÃO PELA GARRA (DESATIVADA PARA TESTE)
    # =====================================

    # def garra_callback(self, msg):
    #     if msg.data != 'ajuste' and not msg.data.startswith('ajuste:'):
    #         return
    #
    #     if self.estado != "AGUARDANDO":
    #         return
    #
    #     self.comando_atual = msg.data
    #     self.distancia_atual = None
    #     self.distancia_esquerda = None
    #     self.distancia_direita = None
    #     self.ultima_leitura = None
    #     self.verificacoes = 0
    #     self.inicio_ajuste = self.get_clock().now()
    #     self.estado = "APROXIMANDO"
    #
    #     self.get_logger().info(
    #         'Comando recebido. Iniciando aproximação e alinhamento pelo LiDAR.'
    #     )

    # =====================================
    # LEITURA DO LIDAR
    # =====================================

    def callback(self, msg):
        if not msg.ranges or msg.angle_increment == 0.0:
            return

        quantidade = len(msg.ranges)
        abertura = abs(msg.angle_increment) * quantidade

        # O cálculo com índices circulares exige uma varredura de aproximadamente 360°
        if abs(abertura - 2.0 * math.pi) > math.radians(5.0):
            self.distancia_atual = None
            self.distancia_esquerda = None
            self.distancia_direita = None
            return

        # LiDAR de cabeça para baixo: angle_max aponta para frente
        indice_frente = quantidade - 1
        deslocamento = round(
            math.radians(self.angulo_lateral) / abs(msg.angle_increment)
        )

        # Na montagem invertida, o sentido angular do sensor é espelhado
        # Hipótese inicial: índices menores que a frente apontam para a esquerda
        indice_esquerda = (indice_frente - deslocamento) % quantidade
        indice_direita = (indice_frente + deslocamento) % quantidade

        distancia_frente = msg.ranges[indice_frente]
        distancia_esquerda = msg.ranges[indice_esquerda]
        distancia_direita = msg.ranges[indice_direita]

        distancias = (
            distancia_frente,
            distancia_esquerda,
            distancia_direita
        )

        if not all(
            math.isfinite(d) and msg.range_min <= d <= msg.range_max
            for d in distancias
        ):
            self.distancia_atual = None
            self.distancia_esquerda = None
            self.distancia_direita = None
            return

        self.distancia_atual = distancia_frente
        self.distancia_esquerda = distancia_esquerda
        self.distancia_direita = distancia_direita
        self.ultima_leitura = self.get_clock().now()

    # =====================================
    # CONTROLE PRINCIPAL
    # =====================================

    def controlar(self):
        if self.estado in ("AGUARDANDO", "ERRO"):
            return

        agora = self.get_clock().now()
        cmd = Twist()

        tempo_ajuste = (
            agora - self.inicio_ajuste
        ).nanoseconds / 1e9

        if tempo_ajuste > self.timeout_ajuste:
            self.erro_ajuste("Tempo máximo de ajuste excedido")
            return

        if self.ultima_leitura is None:
            self.parar()
            # Aguarda uma leitura válida, sem continuar movimentando
            tempo_sem_leitura = tempo_ajuste
            if tempo_sem_leitura > self.timeout:
                self.erro_ajuste("Sem leitura válida do LiDAR")
            return

        tempo_scan = (
            agora - self.ultima_leitura
        ).nanoseconds / 1e9

        if tempo_scan > self.timeout:
            self.erro_ajuste("Timeout do LiDAR")
            return

        # Não utiliza valores inválidos após uma leitura descartada
        if any(d is None for d in (
            self.distancia_atual, self.distancia_esquerda, self.distancia_direita
        )):
            self.parar()
            return

        distancia = self.distancia_atual
        erro_lateral = self.distancia_esquerda - self.distancia_direita

        if distancia < self.distancia_minima_segura:
            self.erro_ajuste("Mesa próxima demais do LiDAR")
            return

        # ETAPA 1: APROXIMAR
        if self.estado == "APROXIMANDO":
            if distancia > self.distancia_alvo + self.tolerancia:
                cmd.linear.x = self.velocidade_frente

            elif distancia < self.distancia_alvo - self.tolerancia:
                cmd.linear.x = -self.velocidade_ajuste

            else:
                self.parar()
                self.estado = "ALINHANDO"

                self.get_logger().info(
                    'Distância alcançada. Iniciando alinhamento.'
                )
                return

        # ETAPA 2: ALINHAR
        elif self.estado == "ALINHANDO":
            if abs(erro_lateral) <= self.tolerancia_lateral:
                self.parar()
                self.estado = "VERIFICANDO"
                self.verificacoes = 0

                self.get_logger().info(
                    'Alinhamento alcançado. Verificando distância e orientação.'
                )
                return

            velocidade = self.kp_rotacao * erro_lateral

            velocidade = max(
                -self.velocidade_angular_max,
                min(self.velocidade_angular_max, velocidade)
            )

            if abs(velocidade) < self.velocidade_angular_min:
                velocidade = math.copysign(
                    self.velocidade_angular_min, velocidade
                )

            cmd.angular.z = self.sinal_rotacao * velocidade

        # ETAPA 3: VERIFICAR
        elif self.estado == "VERIFICANDO":
            distancia_correta = (
                abs(distancia - self.distancia_alvo) <= self.tolerancia
            )

            alinhamento_correto = (
                abs(erro_lateral) <= self.tolerancia_lateral
            )

            if not alinhamento_correto:
                self.verificacoes = 0
                self.estado = "ALINHANDO"
                self.parar()
                return

            if not distancia_correta:
                self.verificacoes = 0
                self.estado = "APROXIMANDO"
                self.parar()
                return

            self.verificacoes += 1

            if self.verificacoes >= self.verificacoes_necessarias:
                self.finalizar_ajuste()
                return

        self.publisher.publish(cmd)

        self.get_logger().debug(
            f'Estado: {self.estado} | '
            f'Frente: {self.distancia_atual:.3f} | '
            f'Esquerda: {self.distancia_esquerda:.3f} | '
            f'Direita: {self.distancia_direita:.3f} | '
            f'Erro lateral: {erro_lateral:.3f} | '
            f'Vx: {cmd.linear.x:.2f} | '
            f'Wz: {cmd.angular.z:.2f}'
        )

    # =====================================
    # FINALIZAÇÃO
    # =====================================

    def finalizar_ajuste(self):
        self.parar()
        self.estado = "AGUARDANDO"

        # msg = String()
        # msg.data = self.comando_atual
        # self.publisher_ajuste.publish(msg)

        self.get_logger().info('Ajuste concluído.')

        self.comando_atual = None
        self.inicio_ajuste = None

    # =====================================
    # ERRO
    # =====================================

    def erro_ajuste(self, motivo):
        self.parar()
        self.estado = "ERRO"

        self.get_logger().error(
            f'Ajuste interrompido: {motivo}'
        )

        # msg = String()
        # msg.data = "erro no ajuste"
        # self.publisher_ajuste.publish(msg)

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