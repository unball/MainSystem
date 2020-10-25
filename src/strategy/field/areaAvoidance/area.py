import sys
sys.path.append("src/")

from abc import ABC, abstractmethod
import numpy as np
from scipy.optimize import shgo, brute
from tools import unit, norml

class Area:
    def __init__(self):
        super().__init__()

    @abstractmethod
    def P(self, t: float):
        pass

    def nearestTo(self, P):
        # argmin_t |self.P(t) - P|
        P = np.array(P)[:2]
        optimizer = brute(lambda t: norml(self.P(t[0]) - P), [(0,1)])
        return round(optimizer[0], 2)

    def dP(self, t, d=0.001):
        return (self.P((t+d) % 1) - self.P(t)) / d

    def normalTo(self, P):
        tn = self.nearestTo(P)
        dP = self.dP(tn)

        normal = np.array([dP[1], -dP[0]])

        return normal / norml(normal)

    def distanceTo(self, P):
        P = np.array(P)[:2]
        Pn = self.P(self.nearestTo(P))
        n = self.normalTo(P)

        return np.dot(P - Pn, n)

class AvoidCircle(Area):
    def __init__(self, center, radius):
        super().__init__()
        self.center = np.array(center)
        self.radius = radius

    def P(self, t):
        return self.center + self.radius*unit(2*np.pi*t)