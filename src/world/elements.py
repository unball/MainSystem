import time
from tools.interval import Interval
from tools import adjustAngle, norml, derivative, angularDerivative
import numpy as np
import math

class EntriesVec:
    def __init__(self, past=20):
        self.past = 20
        self.vec = [0] * self.past

    def add(self, value):
        self.vec = [value] + self.vec[0:self.past-1]

    @property
    def value(self):
        return self.vec[0]

class Element:
    def __init__(self):
        self.xvec = EntriesVec()
        self.yvec = EntriesVec()
        self.interval = Interval(initial_dt=0.016)

    def update(self, x, y):
        self.xvec.add(x)
        self.yvec.add(y)
        self.interval.update()

    @property
    def x(self):
        return self.xvec.value

    @property
    def y(self):
        return self.yvec.value

    @property
    def pos(self):
        return (self.x, self.y)

    @property
    def vx(self):
        return derivative(self.xvec.vec, self.interval.dt)

    @property
    def vy(self):
        return derivative(self.yvec.vec, self.interval.dt)

    @property
    def v(self):
        return (self.vx, self.vy)

    @property
    def velmod(self):
        return norml(self.v)

class Robot(Element):
    def __init__(self, id):
        super().__init__()
        self.id = id
        self.thvec_raw = EntriesVec()

    def update(self, x, y, th):
        self.thvec_raw.add(th)
        super().update(x,y)

class TeamRobot(Robot):
    def __init__(self, id):
        super().__init__(id)

        self.field = None
        self.vref = math.inf
        self.spin = 0
        self.timeLastResponse = None
        self.lastControlLinVel = 0
        self.direction = 1

    def updateField(self, field):
        self.field = field

    @property
    def thvec(self):
        return [th + (np.pi if self.direction == -1 else 0) for th in self.thvec_raw.vec]

    @property
    def th(self):
        return self.thvec[0]

    @property
    def pose(self):
        return (self.x, self.y, self.th)

    @property
    def w(self):
        return angularDerivative(self.thvec.vec, self.interval.dt)
    
    def isAlive(self):
        """Verifica se o robô está vivo baseado na relação entre a velocidade enviada pelo controle e a velocidade medida pela visão"""
        ctrlVel = np.abs(self.lastControlLinVel)
        
        if ctrlVel < 0.01:
            self.timeLastResponse = time.time()
            return True
        
        if self.velmod / ctrlVel < 0.1:
            dt = time.time() - self.timeLastResponse
            if dt is not None and dt > 0.33:
                return False
        else:
            self.timeLastResponse = time.time()
        
        return True

class Ball(Element):
    def __init__(self):
        super().__init__()
