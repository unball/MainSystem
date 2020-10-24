from client import VSS
from world import World
from control.UFC import UFC
from strategy import Strategy
import time

vss = VSS()
vss_enemy = VSS(team_yellow=True)

class Loop:
    def __init__(self, loopFreq = 60):
        self.world = World(3)
        # self.world.enemies[0].control = UFC()
        # self.world.enemies[1].control = UFC()
        # self.world.enemies[2].control = UFC()
        self.loopTime = 1.0 / loopFreq
        self.running = True
        self.strategy = Strategy(self.world)

    def loop(self):
        # Executa visão
        message = vss.vision.read()
        if message is None: return

        # Atualiza o estado de jogo
        self.world.update(message)

        # Executa estratégia
        self.strategy.update()

        # Executa o controle
        vss.command.writeMulti([robot.control.actuate(robot) for robot in self.world.team])

    def run(self):
        while self.running:
            # Tempo inicial do loop
            t0 = time.time()

            # Executa o loop
            self.loop()

            # Dorme para que a próxima chamada seja 
            time.sleep(max(self.loopTime - (time.time()-t0), 0))

# Instancia o programa principal
loop = Loop()

loop.run()