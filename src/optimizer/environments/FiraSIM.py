from abc import abstractmethod
from optimizer.environment import Environment
from client import VSS
import numpy as np
import time

class FiraSIM(Environment):
    def __init__(self, loopRate=0.016):
        Environment.__init__(self)

        self.simulator = VSS()
        self.simulatorYellow = VSS(team_yellow=True)
        self.loopRate = loopRate
    
    @abstractmethod
    def setInitialConditions(self):
        pass
    
    def execute(self, action):
        self.simulator.command.write(0, *action)
        #time.sleep(self.loopRate)
    
    @property
    def state(self):
        return self.simulator.vision.read()

class CornerSituation(FiraSIM):
    def __init__(self, loopRate=0.016):
        FiraSIM.__init__(self, loopRate)
    
    def setInitialConditions(self):
        for i in range(0,3):
            self.simulator.command.setPos(i, 1, 1, 0)
            self.simulatorYellow.command.setPos(i, 1, 1, 0)
        
        self.simulator.command.setBallPos(-0.5, 0.5)
        self.simulator.command.setPos(0, 0.5, -0.5, -np.pi)