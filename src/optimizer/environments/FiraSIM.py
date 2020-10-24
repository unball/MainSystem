from optimizer.environment import Environment
from client import VSS
import numpy as np
import time

class FiraSIM(Environment):
    def __init__(self, loopRate=0.016):
        Environment.__init__(self)

        self.simulator = VSS()
        self.loopRate = loopRate
    
    def setInitialConditions(self):
        self.simulator.command.setBallPos(-0.5, 0.5)
        self.simulator.command.setPos(0, 0.5, -0.5, -np.pi)
    
    def execute(self, action):
        self.simulator.command.write(0, *action)
        #time.sleep(self.loopRate)
    
    @property
    def state(self):
        return self.simulator.vision.read()

def CornerSituation(FiraSIM):
    def __init__(self, loopRate=0.016):
        FiraSIM.__init__(self, loopRate)
    
    def setInitialConditions(self):
        self.simulator.command.setBallPos(-0.5, 0.5)
        self.simulator.command.setPos(0, 0.5, -0.5, -np.pi)