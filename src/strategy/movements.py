import numpy as np
from tools import unit, angl, ang


def goToBall(rb, rg):
    offset = -0.015 * unit(angl(rg-rb)) #+ 0.015 * unit(angl(rg-rb) + np.pi/2)

    rb = rb + offset
    
    # Ângulo da bola até o gol
    angle = ang(rb, rg)

    return np.array([*rb[:2], angle])