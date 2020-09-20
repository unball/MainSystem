
import numpy as np
from tools import angl, unit, norml, angError, filt, sat
from . import Field

class DirectionalField(Field):
    def __init__(self, th, nullgamma=True):
        super().__init__(nullgamma=nullgamma)
        self.th = th

    def F(self, P):
        return self.th
    