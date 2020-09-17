import time
from tools.interval import Interval
from tools import adjustAngle

class Robot:
    def __init__(self, id):
        self.id = id
        self.past = 3

        self.xvec = [0]*self.past
        self.yvec = [0]*self.past
        self.thvec = [0]*self.past

        self.intervalManager = Interval()

    def update(self, x, y, th):
        self.xvec =   [x] + self.xvec[0:self.past-1]
        self.xvec =   [x] + self.xvec[0:self.past-1]
        self.yvec =   [y] + self.yvec[0:self.past-1]
        self.thvec = [th] + self.thvec[0:self.past-1]

        self.dt = self.intervalManager.getInterval()

    @property
    def x(self):
        return self.xvec[0]

    @property
    def y(self):
        return self.yvec[0]

    @property
    def th(self):
        return self.thvec[0]

    @property
    def pose(self):
        return (self.x, self.y, self.th)

    @property
    def w(self):
        if self.dt is None: return 0
        return (self.thvec[0] - self.thvec[1]) / self.dt

    @property
    def alpha(self):
        if self.dt is None: return 0
        return adjustAngle(self.thvec[0] - 2 * self.thvec[1] + self.thvec[2]) / self.dt**2

class TeamRobot(Robot):
    def __init__(self, id):
        super().__init__(id)
        self.field = None

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
