from world import World
from strategy.strategies.singleAttacker import SingleAttacker
from optimizer.system import System
from control.UFC import UFC_Simple
from tools import norm
import numpy as np
import time

class Attacker(System):
    def __init__(self, params, experimentTimeout=0.3):
        System.__init__(self)

        self.params = params
        self.experimentTimeout = experimentTimeout
        self.experimentStart = None

        self.world = World()

        self.strategy = SingleAttacker(self.world, self.paramsDict(["perpBallLimiarTrackState", "perpBallLimiarAtackState", "alignmentAngleTrackState", "alignmentAngleAtackState", "spiralRadius", "spiralRadiusCorners", "approximationSpeed", "ballOffset"]))

        self.world.team[0].control = UFC_Simple(**self.paramsDict(["kw", "kp", "mu", "vmax", "L"]))
    
    def action(self, state):
        if self.experimentStart is None: self.experimentStart = time.time()
        if state is None: return (0,0)

        # Atualiza o estado de jogo
        self.world.update(state)

        # Executa estratégia
        self.strategy.update()

        robot = self.world.team[0]

        return robot.control.actuate(robot)
    
    @property
    def metrics(self):
        ballDistance = norm(self.world.ball.pos, self.world.team[0].pos)
        ballGoalDistance = norm(self.world.ball.pos, self.world.field.goalPos)
        
        return [self.world.team[0].control.error, ballDistance, ballGoalDistance]

    @staticmethod
    def getParams():
        return {
            #"perpBallLimiarTrackState": [0,1], 
            #"perpBallLimiarAtackState": [0,1], 
            #"alignmentAngleTrackState": [0,90], 
            #"alignmentAngleAtackState": [0,90], 
            "spiralRadius": [0,0.2], 
            "spiralRadiusCorners": [0,0.2], 
            "approximationSpeed": [0,1], 
            "ballOffset": [0,0.1],
            "kw": [0,10],
            "kp": [0,10],
            "mu": [0,1],
            "vmax": [0,2],
            #"L": [0,0.1]
        }

    def cost(self, experimentMetrics):
        experimentMetrics = np.array(experimentMetrics)
        controlErrorCost = np.abs(experimentMetrics[:,0]).mean()
        ballDistanceCost = experimentMetrics[:,1].mean()
        ballGoalDistanceCost = experimentMetrics[:,2].min()

        return controlErrorCost + ballDistanceCost + ballGoalDistanceCost

    def endExperiment(self):
        if self.experimentStart is None: return False
        return time.time() - self.experimentStart > self.experimentTimeout

    def param(self, key):
        keylist = list(Attacker.getParams().keys())
        if key not in keylist: return None
        else: return self.params[keylist.index(key)]

    def paramsDict(self, keys):
        pd = {}
        for k in keys:
            value = self.param(k)
            if value is not None: pd[k] = value
        
        return pd