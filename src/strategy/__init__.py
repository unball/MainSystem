from strategy.field import UVF
from strategy.movements import goToBall
import numpy as np

class Strategy:
    def __init__(self, world):
        self.world = world
    
    def update(self):
        rb = np.array(self.world.ball.pos)
        rg = np.array((0.65, 0))
        Pb = goToBall(rb, rg)
        self.world.team[0].field = UVF(Pb)