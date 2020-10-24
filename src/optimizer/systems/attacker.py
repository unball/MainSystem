from world import World
from strategy.strategies.singleAttacker import SingleAttacker
from optimizer.system import System
from control.UFC import UFC_Simple
from tools import norm
import numpy as np
import time

class Attacker(System):
    def __init__(self, params, experimentTimeout=1):
        System.__init__(self)

        self.params = params
        self.experimentTimeout = experimentTimeout
        self.experimentStart = None

        self.world = World()

        self.strategy = SingleAttacker(self.world, {
            "perpBallLimiarTrackState": self.param("perpBallLimiarTrackState"),
            "perpBallLimiarAtackState": self.param("perpBallLimiarAtackState"),
            "alignmentAngleTrackState": self.param("alignmentAngleTrackState"),
            "alignmentAngleAtackState": self.param("alignmentAngleAtackState"),
            "spiralRadius": self.param("spiralRadius"),
            "spiralRadiusCorners": self.param("spiralRadiusCorners"),
            "approximationSpeed": self.param("approximationSpeed"),
            "ballOffset": self.param("ballOffset")
        })

        self.world.team[0].control = UFC_Simple(kw=self.param("kw"), kp=self.param("kp"), mu=self.param("mu"), vmax=self.param("vmax"), L=self.param("L"))
    
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
        if ballDistance < 0.01: 
            nearBall = True
        else:
            nearBall = False
        
        return [self.world.ball.x, self.world.team[0].control.error, ballDistance, nearBall]

    @staticmethod
    def getParams():
        return {
            "perpBallLimiarTrackState": [0,1], 
            "perpBallLimiarAtackState": [0,1], 
            "alignmentAngleTrackState": [0,90], 
            "alignmentAngleAtackState": [0,90], 
            "spiralRadius": [0,0.2], 
            "spiralRadiusCorners": [0,0.2], 
            "approximationSpeed": [0,1], 
            "ballOffset": [0,0.1],
            "kw": [0,10],
            "kp": [0,10],
            "mu": [0,1],
            "vmax": [0,2],
            "L": [0,0.1]
        }

    def cost(self, experimentMetrics):
        experimentMetrics = np.array(experimentMetrics)
        ballCost = 0.65 - experimentMetrics[:,0].mean()
        controlErrorCost = np.abs(experimentMetrics[:,1]).mean()
        ballDistanceCost = experimentMetrics[:,2].mean()

        return ballCost + controlErrorCost + ballDistanceCost

    def endExperiment(self):
        if self.experimentStart is None: return False
        return time.time() - self.experimentStart > self.experimentTimeout

    def param(self, key):
        return self.params[list(Attacker.getParams().keys()).index(key)]