from client import VSS
from world import World
from control.UFC import UFC
from strategy import Strategy
from UVF_screen import UVFScreen
import time

vss = VSS()
vss_enemy = VSS(team_yellow=True)

class Loop:
    def __init__(self, loopFreq = 60, draw_UVF = False):
        self.world = World(5)
        self.world.enemies[0].control = UFC()
        self.world.enemies[1].control = UFC()
        self.world.enemies[2].control = UFC()
        self.loopTime = 1.0 / loopFreq
        self.running = True
        self.strategy = Strategy(self.world)

        self.draw_UVF = draw_UVF
        if self.draw_UVF:
            self.UVF_screen = UVFScreen(self.world, index_uvf_robot=2)

    def loop(self):
        # Executa visão
        message = vss.vision.read()
        if message is None: return

        # Atualiza o estado de jogo
        self.world.update(message)

        # Executa estratégia
        self.strategy.update()

        # Executa o controle
        vss.command.write(0, *self.world.team[0].control.actuate(self.world.team[0]))
        #for i,robot in enumerate(self.world.team):
        #    vss.command.write(i, *robot.control.actuate(robot))

        #for i,robot in enumerate(self.world.enemies):
        #    vss_enemy.command.write(i, *robot.control.actuate(robot))

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
loop = Loop(draw_UVF=True)

loop.run()