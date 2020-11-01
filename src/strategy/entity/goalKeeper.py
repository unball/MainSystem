from ..entity import Entity
from strategy.field.UVF import UVF
from strategy.field.DirectionalField import DirectionalField
from strategy.field.goalKeeper import GoalKeeperField
from strategy.movements import goalkeep, spinGoalKeeper
from tools import angError, howFrontBall, howPerpBall, ang, norml, norm, angl
from tools.interval import Interval
from control.goalKeeper import GoalKeeperControl
import numpy as np
import math
import time

class GoalKeeper(Entity):
    def __init__(self, world, robot, side=1):
        super().__init__(world, robot)

        self._control = GoalKeeperControl(world)
        self.lastChat = 0
        self.state = "Stable"

    @property
    def control(self):
        return self._control
        
    def directionDecider(self):
       if self.robot.field is not None:
            ref_th = self.robot.field.F(self.robot.pose)
            rob_th = self.robot.th
            # print('Rob_th:', rob_th)

            if norm(self.robot.pos, self.world.ball.pos) < 0.07:
                self.robot.setSpin(np.sign(self.robot.y - self.world.ball.y), timeOut=0.1)

            if abs(angError(ref_th, rob_th)) > 90 * np.pi / 180: #and time.time()-self.lastChat > .3:
                self.robot.direction *= -1
                self.lastChat = time.time()
            
            # Inverter a direção se o robô ficar preso em algo
            elif not self.robot.isAlive() and self.robot.spin == 0:
                if time.time()-self.lastChat > .3:
                    self.lastChat = time.time()
                    self.robot.direction *= -1

    def fieldDecider(self):
        rr = np.array(self.robot.pos)
        vr = np.array(self.robot.v)
        # self.vravg = 0.995 * self.vravg + 0.005 * norml(vr)
        rb = np.array(self.world.ball.pos)
        vb = np.array(self.world.ball.v)
        rg = -np.array(self.world.field.goalPos)
        rg[0] += 0.2
    
         # Aplica o movimento
        self.robot.vref = 0

        self.robot.setSpin(spinGoalKeeper(rb, rr, rg), timeOut = 0.1)

        Pb = goalkeep(rb, vb, rr, rg)

        if self.state == "Stable":
            self.state = "Stable" if np.abs(Pb[0]-rr[0]) < 0.06 else "Unstable"
        else:
            self.state = "Unstable" if np.abs(Pb[0]-rr[0]) > 0.02 else "Stable"

        #self.robot.field = UVF(Pb, spiral=0.01)
        #self.robot.field = DirectionalField(Pb[2], Pb=Pb) if np.abs(rr[0]-Pb[0]) < 0.07 else UVF(Pb, spiral=0.01)

        if self.state == "Stable":
            self.robot.field = DirectionalField(Pb[2], Pb=(rr[0], Pb[1], Pb[2]))
        else:
            self.robot.field = UVF(Pb, spiral=0.02)
        #self.robot.field = DirectionalField(Pb[2], Pb=Pb)