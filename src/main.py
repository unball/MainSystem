from client import VSS
from world import World
from strategy import Strategy
from UVF_screen import UVFScreen
import time

vss = VSS()
vss_enemy = VSS(team_yellow=True)

class Loop:
    def __init__(self, loopFreq = 60, draw_UVF = False):
        self.world = World(3, side=1)
        self.loopTime = 1.0 / loopFreq
        self.running = True
        self.strategy = Strategy(self.world)

        self.draw_UVF = draw_UVF
        if self.draw_UVF:
            self.UVF_screen = UVFScreen(self.world, index_uvf_robot=1)

    def loop(self):
        # Executa visão
        message = vss.vision.read()
        if message is None: return

        # Atualiza o estado de jogo
        self.world.update(message)

        # Executa estratégia
        self.strategy.update()

        # Executa o controle
        vss.command.writeMulti([robot.entity.control.actuate(robot) for robot in self.world.team if robot.entity is not None])

        if self.draw_UVF:
            self.UVF_screen.updateScreen()

        if self.draw_UVF:
            self.UVF_screen.updateScreen()

    def run(self):

        if self.draw_UVF:
            self.UVF_screen.initialiazeScreen()
            self.UVF_screen.initialiazeObjects()

        while self.running:
            # Tempo inicial do loop
            t0 = time.time()

            # Executa o loop
            self.loop()

            # Dorme para que a próxima chamada seja 
            time.sleep(max(self.loopTime - (time.time()-t0), 0))

# Instancia o programa principal
loop = Loop(draw_UVF=False)

loop.run()