from .area import Area
from tools import unit
import numpy as np

class AvoidCircle(Area):
    def __init__(self, center, radius):
        super().__init__()
        self.center = np.array(center)
        self.radius = radius

    def P(self, t):
        return self.center + self.radius*unit(2*np.pi*t)