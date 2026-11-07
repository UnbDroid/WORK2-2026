
import math
import rclpy

from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class AproximarRotacionar(Node):

    def __init__(self):
        super().__init__('aproximar_rotacionar')

        # Publishers
        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.publisher_ajuste = self.create_publisher(
            String, '/topico_ajuste', 10
        )

        # Subscribers
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            qos_profile_sensor_data
        )

        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.callback_odom,
            qos_profile_sensor_data
        )

        self.sub_garra = self.create_subscription(
            String,
            '/topico_garra',
            self.garra_callback,
            10
        )

        # Configurações da aproximação
        self.distancia_alvo = 0.20
        self.tolerancia = 0.02
        self.velocidade_frente = 0.2

        # Configurações da rotação
        self.angulo_alvo = 0.0  # graus
        self.tolerancia_angular = 1.0

        self.kp_rotacao = 1.0
        self.velocidade_angular_max = 0.7 #0.5
        self.velocidade_angular_min = 0.3 #mudei 0.15

        # Odometria
        self.angulo_atual = None
        self.ultima_odom = None

        # LiDAR
        self.distancia_atual = None
        self.ultima_leitura = None

        # Estados
        self.estado = "AGUARDANDO"

        # Segurança
        self.timeout = 2.0

        # Timer de controle: 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.controlar
        )

        self.get_logger().info(
            'Nó iniciado. Aguardando comando da garra.'
        )

    # =====================================
    # ATIVAÇÃO PELA GARRA
    # =====================================

    def garra_callback(self, msg):

        if not msg.data.startswith('transladar:'):
            return

        if self.estado != "AGUARDANDO":
            return

        self.distancia_atual = None
        self.ultima_leitura = None

        self.estado = "APROXIMANDO"

        self.get_logger().info(
            'Comando recebido. Iniciando aproximação.'
        )

    # =====================================
    # LEITURA DO LIDAR
    # =====================================

    def callback(self, msg):

        if not msg.ranges:
            return

        # LiDAR invertido: último raio aponta para frente
        distancia = msg.ranges[-1]

        if not math.isfinite(distancia):
            return

        if not msg.range_min <= distancia <= msg.range_max:
            return

        self.distancia_atual = distancia
        self.ultima_leitura = self.get_clock().now()

    # =====================================
    # LEITURA DA ODOMETRIA
    # =====================================

    def callback_odom(self, msg):

        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y**2 + q.z**2)
        )

        self.angulo_atual = math.degrees(yaw)
        self.ultima_odom = self.get_clock().now()

    # =====================================
    # CONTROLE PRINCIPAL
    # =====================================

    def controlar(self):

        if self.estado in ("AGUARDANDO", "ERRO"):
            return

        agora = self.get_clock().now()
        cmd = Twist()

        # ETAPA 1: APROXIMAR
        if self.estado == "APROXIMANDO":

            if self.ultima_leitura is None:
                # Aguarda primeira leitura válida
                self.parar()
                return

            tempo_scan = (
                agora - self.ultima_leitura
            ).nanoseconds / 1e9

            if tempo_scan > self.timeout:
                self.erro_ajuste("Timeout do LiDAR")
                return

            distancia = self.distancia_atual

            if distancia > self.distancia_alvo + self.tolerancia:
                cmd.linear.x = self.velocidade_frente

            elif distancia < self.distancia_alvo - self.tolerancia:
                cmd.linear.x = -self.velocidade_frente

            else:
                self.parar()
                self.estado = "ROTACIONANDO"

                self.get_logger().info(
                    'Distância alcançada. Iniciando rotação.'
                )
                return

        # ETAPA 2: ROTACIONAR
        elif self.estado == "ROTACIONANDO":

            if self.angulo_atual is None or self.ultima_odom is None:
                self.parar()
                return

            tempo_odom = (
                agora - self.ultima_odom
            ).nanoseconds / 1e9

            if tempo_odom > self.timeout:
                self.erro_ajuste("Timeout da odometria")
                return

            # Erro normalizado entre -180 e 180 graus
            erro = (
                self.angulo_alvo - self.angulo_atual + 180
            ) % 360 - 180

            if abs(erro) <= self.tolerancia_angular:
                self.finalizar_ajuste()
                return

            velocidade = self.kp_rotacao * math.radians(erro)

            velocidade = max(
                -self.velocidade_angular_max,
                min(self.velocidade_angular_max, velocidade)
            )

            if abs(velocidade) < self.velocidade_angular_min:
                velocidade = math.copysign(
                    self.velocidade_angular_min, erro
                )

            cmd.angular.z = velocidade

        self.publisher.publish(cmd)

        self.get_logger().debug(
            f'Estado: {self.estado} | '
            f'Distância: {self.distancia_atual} | '
            f'Ângulo: {self.angulo_atual} | '
            f'Vx: {cmd.linear.x:.2f} | '
            f'Wz: {cmd.angular.z:.2f}'
        )

    # =====================================
    # FINALIZAÇÃO
    # =====================================

    def finalizar_ajuste(self):

        self.parar()
        self.estado = "AGUARDANDO"

        msg = String()
        msg.data = "terminou de ajustar"

        self.publisher_ajuste.publish(msg)

        self.get_logger().info(
            'Ajuste concluído. Visão liberada.'
        )

    # =====================================
    # ERRO
    # =====================================

    def erro_ajuste(self, motivo):

        self.parar()
        self.estado = "ERRO"

        self.get_logger().error(
            f'Ajuste interrompido: {motivo}'
        )

        msg = String()
        msg.data = "erro no ajuste"
        self.publisher_ajuste.publish(msg)

    # =====================================
    # PARADA
    # =====================================

    def parar(self):
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
