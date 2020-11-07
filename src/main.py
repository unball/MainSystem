from client import VSS
from world import World
from strategy import MainStrategy, EnemyStrategy
from UVF_screen import UVFScreen
from client.referee import RefereeCommands, RefereePlacement
import time

vss = VSS()
vss_enemy = VSS(team_yellow=True)
rc = RefereeCommands('224.5.23.2', 10003)
rp = RefereePlacement('224.5.23.2', 10004)
rp_enemy = RefereePlacement('224.5.23.2', 10004, True)

class Loop:
    def __init__(self, loopFreq = 60, draw_UVF = False):
        self.world = World(3, side=1)
        self.enemyWorld = World(3, side=-1)

        self.strategy = MainStrategy(self.world)
        self.enemyStrategy = EnemyStrategy(self.enemyWorld)

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
        self.enemyWorld.update(vss.vision.invertMessage(message))

        # Executa estratégia
        self.strategy.manageReferee(rp, command)
        self.strategy.update()
        self.enemyStrategy.manageReferee(rp_enemy, command)
        self.enemyStrategy.update()

        # Executa o controle
        vss.command.writeMulti([robot.entity.control.actuate(robot) for robot in self.world.team if robot.entity is not None])
        vss_enemy.command.writeMulti([robot.entity.control.actuate(robot) for robot in self.enemyWorld.team if robot.entity is not None])

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