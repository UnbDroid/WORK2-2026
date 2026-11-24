import re
import rclpy
from .gripper_primitives import GripperPrimitives

# ============================== CONFIGURAÇÃO ==============================
# Cavidade (zona) onde cada cubo deve ficar. O nome é o que o nó de visão entende em
# "transladar:<nome>": precisa começar com 'zone' (o número é o id da AprilTag da cavidade).
ZONA_DO_CUBO = {1: 'zone1', 2: 'zone2', 3: 'zone3', 4: 'zone4', 5: 'zone5', 6: 'zone6'}
ZONAS = list(ZONA_DO_CUBO.values())

LIBERAR_CAMERA_APOS_DEIXAR = False  # True: gira o braço para o chassi depois de soltar (se ele tapar a câmera)
MAX_VISITAS = 20                    # trava de segurança contra loop infinito
PAUSA_ANTES_DE_PUBLICAR = 1.0       # s (era o time.sleep(1) antes de ajustar/transladar)

TIMEOUT_GARRA = 30.0                # s sem eco num comando da garra
TIMEOUT_AJUSTE = 60.0               # s sem eco no ajustar
TIMEOUT_VISAO = 150.0               # s sem eco no transladar (a busca pode varrer vários trechos)
TIMEOUT_INSPECAO = 20.0             # s sem resposta no inspecionar
# ==========================================================================


class FalhaPasso(Exception):
    """Algo impediu de continuar (sem eco, resposta inválida, dado faltando)."""


class CavityAction(GripperPrimitives):
    """
    Organiza os cubos nas cavidades. O fluxo é uma única rotina (gerador): cada
    'yield' devolve o controle ao do_work() enquanto espera um eco, então nada bloqueia.

    A cada visita a uma cavidade o nó de visão responde (inspecionar):
      certo  -> o cubo já está no lugar: ignora e vai para outra cavidade
      vazio  -> não pega nada; se estiver carregando um cubo, deixa ele aqui
      cubeN  -> cubo errado: pega (slot livre), deixa o que carregava (se houver)
                e segue para a cavidade de origem do cubo que acabou de pegar
    São usados só os slots 1 e 2, sempre revezando: o cubo novo vai para o slot que não
    está ocupado pelo cubo carregado.
    """

    def __init__(self):
        super().__init__('place_at_cavity')  # nome precisa bater com nome da action no pddl domain
        self._zerar()

    def _zerar(self):
        self.gen = None
        self.terminou = False
        self.rot = None            # última rotação comandada: 'frente', 'slot1' ou 'slot2'
        self.resolvidas = set()    # cavidades que já estão com o cubo certo
        self.mensagem_final = 'place-at-cavity concluído'

    def on_activate(self, state):
        self._zerar()              # cada ativação recomeça do zero
        return super().on_activate(state)

    # ------------------------------------------------------------------
    # Passos básicos (cada um corresponde a um estado do enum antigo)
    # ------------------------------------------------------------------
    def progresso(self):
        return min(0.99, len(self.resolvidas) / len(ZONAS))

    def passo(self, cmd, texto, timeout=TIMEOUT_GARRA):
        """Publica 'cmd' e espera o mesmo texto de volta, um tick por vez."""
        self.send_feedback(self.progresso(), texto)
        self.send_command(cmd)
        if cmd in ('frente', 'slot1', 'slot2'):
            self.rot = cmd
        t0 = self.get_clock().now()
        while not self.command_finished():
            if (self.get_clock().now() - t0).nanoseconds / 1e9 > timeout:
                raise FalhaPasso(f"Sem eco para '{cmd}' depois de {timeout:.0f} s")
            yield

    def pausa(self, segundos): #resposável pelo tempo de espera entre ajustar e trasnladar
        t0 = self.get_clock().now()
        while (self.get_clock().now() - t0).nanoseconds / 1e9 < segundos:
            yield

    def ir_para_zona(self, zona, loc, altura):
        """AJUSTAR + TRANSLADAR: alinha o robô em frente à cavidade."""
        yield from self.pausa(PAUSA_ANTES_DE_PUBLICAR)
        yield from self.passo(f'ajustar:{altura}',
                              f'Ajustando distância e ângulo', TIMEOUT_AJUSTE)
        yield from self.pausa(PAUSA_ANTES_DE_PUBLICAR)
        yield from self.passo(f'transladar:{zona}:{altura}:{loc}',
                              f'Posicionando em frente a {zona}', TIMEOUT_VISAO)

    def inspecionar(self, zona):
        """
        Pergunta ao nó de visão o que há na cavidade em frente.
        Publica 'inspecionar:<zona>' e espera '/topico_garra_status' = 'inspecionar:<zona>:<resultado>',
        com resultado 'vazio', 'certo' ou 'cubeN'.
        """
        prefixo = f'inspecionar:{zona}:'
        self.send_feedback(self.progresso(), f'Inspecionando {zona}')
        self.send_command(f'inspecionar:{zona}')
        t0 = self.get_clock().now()
        while True:
            resposta = self._current_reply if self._reply_received else None
            if resposta and resposta.startswith(prefixo):
                return resposta[len(prefixo):].strip().lower()
            if (self.get_clock().now() - t0).nanoseconds / 1e9 > TIMEOUT_INSPECAO:
                raise FalhaPasso(f'Visão não respondeu a inspecionar:{zona}')
            yield

    def pegar(self, slot, altura):
        """PICK_*: pega o cubo da cavidade em frente e guarda no slot indicado."""
        if self.rot != 'frente':   # só levanta e gira se o braço não estiver virado para a mesa
            yield from self.passo('alt_giro', 'Levantando para altura de giro')
            yield from self.passo('frente', 'Girando para pegar o cubo')
        yield from self.passo(altura, f'Descendo até a altura da mesa ({altura})')
        yield from self.passo('fechar', 'Fechando a garra')
        yield from self.passo('alt_giro', 'Subindo com o cubo para altura de giro')
        yield from self.passo(f'slot{slot}', f'Girando para o slot{slot}')
        yield from self.passo('inicial', 'Abaixando para a altura dos slots')
        yield from self.passo('abrir', f'Guardando o cubo no slot{slot}')

    def deixar(self, slot, altura):
        """DEIXAR_*: tira o cubo do slot e coloca na cavidade em frente."""
        yield from self.passo('alt_giro', 'Levantando para altura de giro')
        yield from self.passo(f'slot{slot}', f'Girando para o slot{slot}')
        yield from self.passo('inicial', 'Movendo para a altura dos slots')
        yield from self.passo('fechar', 'Pegando o cubo do slot')
        yield from self.passo('alt_giro', 'Subindo com o cubo para altura de giro')
        yield from self.passo('frente', 'Girando para a mesa')
        yield from self.passo(altura, f'Descendo até a altura da mesa ({altura})')
        yield from self.passo('abrir', 'Soltando o cubo na cavidade')
        yield from self.passo('inicial', 'Subindo para liberar a cavidade')
        if LIBERAR_CAMERA_APOS_DEIXAR:
            yield from self.passo('alt_giro', 'Levantando para girar')
            yield from self.passo('slot2', 'Girando para o chassi (liberar a câmera)')

    # ------------------------------------------------------------------
    # Escolha da próxima cavidade (o "seletor de loc mágica" do diagrama)
    # ------------------------------------------------------------------
    def escolher_proxima(self, vazias):
        candidatas = [z for z in ZONAS if z not in self.resolvidas and z not in vazias]
        return candidatas[0] if candidatas else None   # troque por random.choice(candidatas) se quiser aleatório

    @staticmethod
    def numero_do_cubo(texto):
        m = re.fullmatch(r'cube(\d+)', texto.strip().lower())
        return int(m.group(1)) if m else None

    # ------------------------------------------------------------------
    # Rotina principal
    # ------------------------------------------------------------------
    def rotina(self, loc):
        try:
            altura = self.height_to_cmd(loc)
        except KeyError:
            altura = None
        if altura is None:
            raise FalhaPasso(f'Sem ângulo ou altura para a location {loc}')

        vazias = set()        # cavidades sabidamente vazias (nada para pegar nelas)
        carregando = None     # (cubo, slot) que o robô leva de uma cavidade para a outra
        zona = self.escolher_proxima(vazias)
        visitas = 0

        while zona is not None and len(self.resolvidas) < len(ZONAS):
            visitas += 1
            if visitas > MAX_VISITAS:
                raise FalhaPasso(f'Mais de {MAX_VISITAS} visitas sem terminar')

            yield from self.ir_para_zona(zona, loc, altura)
            estado = yield from self.inspecionar(zona)
            self.get_logger().info(f'{zona}: {estado}')

            # "cubeN" cujo destino é esta mesma cavidade equivale a "certo"
            ocupante = None
            if estado not in ('vazio', 'certo'):
                ocupante = self.numero_do_cubo(estado)
                if ocupante not in ZONA_DO_CUBO:
                    raise FalhaPasso(f"Resposta da visão inválida em {zona}: '{estado}'")
                if ZONA_DO_CUBO[ocupante] == zona:
                    estado, ocupante = 'certo', None

            if estado == 'certo':
                if carregando:
                    raise FalhaPasso(f'{zona} já está certa, mas o robô carrega o cubo {carregando[0]}')
                self.resolvidas.add(zona)                      # SIM: ignora e segue
                zona = self.escolher_proxima(vazias)

            elif estado == 'vazio':
                if carregando:
                    yield from self.deixar(carregando[1], altura)   # só PLACE
                    self.resolvidas.add(zona)
                    carregando = None
                else:
                    vazias.add(zona)                           # nada para pegar nem deixar
                zona = self.escolher_proxima(vazias)

            else:                                              # cubo errado: PICK (+ PLACE)
                if carregando and carregando[0] == ocupante:
                    raise FalhaPasso(f'A visão viu o cubo {ocupante} em {zona}, mas ele está no robô')
                slot_livre = 3 - carregando[1] if carregando else 1   # reveza os slots 1 e 2
                yield from self.pegar(slot_livre, altura)
                if carregando:
                    yield from self.deixar(carregando[1], altura)
                    self.resolvidas.add(zona)
                else:
                    vazias.add(zona)    # esvaziamos a cavidade; o cubo dela vem depois
                carregando = (ocupante, slot_livre)
                zona = ZONA_DO_CUBO[ocupante]
                if zona in self.resolvidas:
                    raise FalhaPasso(f'{zona} já está certa, mas o cubo {ocupante} ainda estava fora dela')

        if carregando:
            raise FalhaPasso(f'Terminou carregando o cubo {carregando[0]}')

        sem_cubo = [z for z in ZONAS if z not in self.resolvidas]
        if sem_cubo:
            self.get_logger().warning(f'Cavidades que ficaram sem cubo: {sem_cubo}')
            self.mensagem_final = f'place-at-cavity concluído (sem cubo: {", ".join(sem_cubo)})'

        yield from self.passo('alt_giro', 'Levantando para a posição de repouso')
        yield from self.passo('slot2', 'Girando garra para a posição de repouso')

    # ------------------------------------------------------------------
    def do_work(self):
        if self.terminou:
            return
        try:
            if self.gen is None:
                self.gen = self.rotina(self.current_arguments[2])  # [robot, cube, location]
            next(self.gen)
        except StopIteration:
            self.terminou = True
            self.finish(True, 1.0, self.mensagem_final)
        except FalhaPasso as e:
            self.terminou = True
            self.get_logger().error(str(e))
            self.finish(False, 0.0, str(e))


def main(args=None):
    rclpy.init(args=args)
    node = CavityAction()
    node.trigger_configure()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()