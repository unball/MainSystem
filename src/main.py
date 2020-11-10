from client import VSS
from world import World
from strategy import MainStrategy, EnemyStrategy
from UVF_screen import UVFScreen
from client.referee import RefereeCommands, RefereePlacement
import time
import sys

import constants

team_yellow = True if sys.argv[1] == "yellow" else False

vss = VSS(constants.HOST_FIRASIM_VISION, constants.PORT_FIRASIM_VISION, constants.HOST_FIRASIM_COMMAND, constants.PORT_FIRASIM_COMMAND, team_yellow=team_yellow)
rc = RefereeCommands(constants.HOST_REFEREE, constants.PORT_REFEREE_COMMAND)
rp = RefereePlacement(constants.HOST_REFEREE, constants.PORT_REFEREE_REPLACEMENT, team_yellow=team_yellow)

#vss_enemy = VSS(team_yellow=True)
#rp_enemy = RefereePlacement(constants.HOST_REFEREE, 10004, True)

class Loop:
    def __init__(self, loopFreq = 60, draw_UVF = False):
        self.world = World(3, side=-1 if team_yellow else 1)
        #self.enemyWorld = World(3, side=-1)

        self.strategy = MainStrategy(self.world)
        #self.enemyStrategy = EnemyStrategy(self.enemyWorld)

        # Variáveis
        self.loopTime = 1.0 / loopFreq
        self.running = True 
        self.draw_UVF = draw_UVF
        if self.draw_UVF:
            self.UVF_screen = UVFScreen(self.world, index_uvf_robot=1)

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
        #vss_enemy.command.writeMulti([robot.entity.control.actuate(robot) for robot in self.enemyWorld.team if robot.entity is not None])

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

# Instancia o programa principal
loop = Loop(draw_UVF=False)

loop.run()