
import numpy as np
from tools import angl, unit, norml, angError, filt, sat
from . import Field

class DirectionalField(Field):
    def __init__(self, th, Pb=None, nullgamma=True):
        super().__init__(nullgamma=nullgamma)
        self.th = th

        if Pb is not None: self.Pb = Pb

    def F(self, P):
        return self.th
    