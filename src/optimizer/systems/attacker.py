from world import World
from strategy.strategies.singleAttacker import SingleAttacker
from control.UFC import UFC_Simple
from tools import norm

class Attacker():
    def __init__(self, params):
        self.params = params

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

    def param(self, key):
        return self.params[list(Attacker.getParams().keys()).index(key)]