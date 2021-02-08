from client import VSS
from world import World
from strategy import MainStrategy, EnemyStrategy
from UVF_screen import UVFScreen
from client.referee import RefereeCommands, RefereePlacement
import matplotlib.pyplot as plt
import time
import sys
import numpy as np

# Verifia a cor do time
if sys.argv[1] == "yellow":
    team_yellow = True
    team_side = -1
else:
    team_yellow = False
    team_side = 1

# Instancia interface com o simulador
vss = VSS(team_yellow=team_yellow)

# Instancia interfaces com o referee
rc = RefereeCommands()
rp = RefereePlacement(team_yellow=team_yellow)

# Classe do programa principal
class Loop:
    def __init__(self, loopFreq = 60, draw_UVF = False):
        self.world = World(3, side=team_side)
        #self.enemyWorld = World(3, side=-1)

        self.strategy = MainStrategy(self.world)
        #self.enemyStrategy = EnemyStrategy(self.enemyWorld)

        # Variáveis
        self.loopTime = 1.0 / loopFreq
        self.running = True

        # Interface gráfica para mostrar campos
        self.draw_UVF = draw_UVF
        if self.draw_UVF:
            self.UVF_screen = UVFScreen(self.world, index_uvf_robot=0)

    def loop(self):
        # Recebe dados do Referee
        command = rc.receive()

        # Executa visão
        message = vss.vision.read()
        if message is None: return

        # Atualiza o estado de jogo
        self.world.update(message)
        #self.enemyWorld.update(vss.vision.invertMessage(message))

        # Executa estratégia
        self.strategy.manageReferee(rp, command)
        self.strategy.update()
        #self.enemyStrategy.manageReferee(rp_enemy, command)
        #self.enemyStrategy.update()

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

            #print((time.time()-t0)*1000)

            # Dorme para que a próxima chamada seja 
            time.sleep(max(self.loopTime - (time.time()-t0), 0))

            #time.sleep(1)

# Instancia o programa principal
loop = Loop(draw_UVF=False)

loop.run()