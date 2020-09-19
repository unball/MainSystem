import time
from tools.interval import Interval
from tools import adjustAngle, norml
import numpy as np
import math

class Robot:
    def __init__(self, id):
        self.id = id
        self.past = 20

        self.xvec = [0]*self.past
        self.yvec = [0]*self.past
        self.thvec_raw = [0]*self.past

        self.lastControlLinVel = 0

        self.direction = 1

        self.intervalManager = Interval()
        self.timeLastResponse = None

    def update(self, x, y, th):
        self.xvec =       [x] + self.xvec[0:self.past-1]
        self.yvec =       [y] + self.yvec[0:self.past-1]
        self.thvec_raw = [th] + self.thvec_raw[0:self.past-1]

        self.dt = self.intervalManager.getInterval()

    @property
    def x(self):
        return self.xvec[0]

    @property
    def y(self):
        return self.yvec[0]

    @property
    def thvec(self):
        return [th + (np.pi if self.direction == -1 else 0) for th in self.thvec_raw]

    @property
    def th(self):
        return self.thvec[0]

    @property
    def pos(self):
        return (self.x, self.y)

    @property
    def pose(self):
        return (self.x, self.y, self.th)

    def derivative(self, vec):
        if self.dt is None: return 0
        return (vec[0] - vec[1]) / self.dt

    @property
    def w(self):
        return self.derivative(self.thvec)

    @property
    def vx(self):
        return self.derivative(self.xvec)

    @property
    def vy(self):
        return self.derivative(self.yvec)

    @property
    def velmod(self):
        vx = self.vx
        vy = self.vy
        if vx is not None and vy is not None:
            return norml((vx, vy))
        else: return 0

    @property
    def alpha(self):
        if self.dt is None: return 0
        return adjustAngle(self.thvec[0] - 2 * self.thvec[1] + self.thvec[2]) / self.dt**2
    
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

class TeamRobot(Robot):
    def __init__(self, id):
        super().__init__(id)
        self.field = None
        self.vref = math.inf
        self.spin = 0

    def updateField(self, field):
        self.field = field

class Ball:
    def __init__(self):
        self.x = 0
        self.y = 0

    def update(self, x, y):
        self.x = x
        self.y = y

    @property
    def pos(self):
        return (self.x, self.y)
