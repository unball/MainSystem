from client import VSS
from world import World
from control.UFC import UFC
from strategy import Strategy
import time

vss = VSS()

class Loop:
    def __init__(self, loopFreq = 60):
        self.world = World(5)
        self.loopTime = 1.0 / loopFreq
        self.running = True
        self.control = UFC()
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
        vl, vr = self.control.actuate(self.world.team[0])
        
        # Envia para os robôs
        #print("{}\t{}".format(vl, vr))
        vss.command.write(0, vl, vr)

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

# Executa o sistema
loop.run()