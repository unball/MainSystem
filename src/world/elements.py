import time
from tools.interval import Interval
from tools import adjustAngle, norml, derivative, angularDerivative, unit, angl
from control.UFC import UFC_Simple
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
    def __init__(self, world):
        self.world = world
        self.xvec = EntriesVec()
        self.yvec = EntriesVec()
        self.interval = Interval(initial_dt=0.016)

    def update(self, x, y):
        self.xvec.add(x)
        self.yvec.add(y)
        self.interval.update()

    @property
    def x_raw(self):
        return self.xvec.value

    @property
    def x(self):
        return self.world.field.side * self.x_raw

    @property
    def y_raw(self):
        return self.yvec.value

    @property
    def y(self):
        return self.y_raw

    @property
    def pos(self):
        return (self.x, self.y)

    @property
    def vx_raw(self):
        return derivative(self.xvec.vec, self.interval.dt)

    @property
    def vx(self):
        return self.world.field.side * self.vx_raw

    @property
    def vy_raw(self):
        return derivative(self.yvec.vec, self.interval.dt)

    @property
    def vy(self):
        return self.vy_raw

    @property
    def v(self):
        return (self.vx, self.vy)

    @property
    def velmod(self):
        return norml(self.v)

class Robot(Element):
    def __init__(self, world, id):
        super().__init__(world)
        self.id = id
        self.thvec_raw = EntriesVec()

    def update(self, x, y, th):
        self.thvec_raw.add(th)
        super().update(x,y)

class TeamRobot(Robot):
    def __init__(self, world, id, control=None):
        super().__init__(world, id)

        self.field = None
        self.vref = math.inf
        self.spin = 0
        self.spinTime = 0
        self.spinTimeOut = 0.5
        self.entity = None
        self.timeLastResponse = None
        self.lastControlLinVel = 0
        self.direction = 1

    def updateField(self, field):
        self.field = field

    def updateEntity(self, entityClass, forced_update=False, **kwargs):
        if type(self.entity) != entityClass or forced_update:
            self.entity = entityClass(self.world, self, **kwargs)

    def setSpin(self, dir=1, timeout=0.25):
        if dir != 0: 
            # Atualiza a direção do spin
            self.spin = dir

            # Atualiza o tempo de início do spin, se for um spin
            self.spinTime = time.time()

            # Diz o tempo de duração do spin
            self.spinTimeOut = timeout
        else:
            self.spin = 0

    @property
    def thvec(self):
        return [th + (np.pi if self.direction == -1 else 0) for th in self.thvec_raw.vec]

    @property
    def th_raw(self):
        return self.thvec[0]

    @property
    def th(self):
        return self.th_raw if self.world.field.side == 1 else adjustAngle(np.pi - self.th_raw)

    @property
    def pose(self):
        return (self.x, self.y, self.th)

    @property
    def w_raw(self):
        return angularDerivative(self.thvec, self.interval.dt)

    @property
    def w(self):
        return self.world.field.side * self.w_raw
    
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
    def __init__(self, world):
        super().__init__(world)
