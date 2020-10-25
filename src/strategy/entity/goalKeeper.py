from ..entity import Entity
from strategy.field.UVF import UVF
from strategy.field.DirectionalField import DirectionalField
from strategy.field.goalKeeper import GoalKeeperField
from strategy.movements import goalkeep, spinGoalKeeper
from tools import angError, howFrontBall, howPerpBall, ang, norml
from tools.interval import Interval
from control.goalKeeper import GoalKeeperControl
import numpy as np
import math

class GoalKeeper(Entity):
    def __init__(self, world, robot, side=1):
        super().__init__(world, robot)

        self._control = GoalKeeperControl(world)

    @property
    def control(self):
        return self._control
        
    def directionDecider(self):
       if self.robot.field is not None:
            ref_th = self.robot.field.F(self.robot.pose)
            rob_th = self.robot.th
            # print('Rob_th:', rob_th)

            if abs(angError(ref_th, rob_th)) > 120 * np.pi / 180:
                self.robot.direction *= -1

    def fieldDecider(self):
        rr = np.array(self.robot.pos)
        vr = np.array(self.robot.v)
        # self.vravg = 0.995 * self.vravg + 0.005 * norml(vr)
        rb = np.array(self.world.ball.pos)
        vb = np.array(self.world.ball.v)
        rg = -np.array(self.world.field.goalPos)
        rg[0] += 0.25
    
         # Aplica o movimento
        self.robot.vref = 0

        self.robot.setSpin(spinGoalKeeper(rb, rr, rg), timeout = 0.1)

        Pb = goalkeep(rb, vb, rr, rg)
        # print('Pb:', Pb)
        self.robot.field = UVF(Pb)
        