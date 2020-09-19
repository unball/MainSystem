import numpy as np
from tools import unit, angl, ang, norm, sat

def goToBall(rb, vb, rg, rr, rl, vravg):
    rbp = rb + vb * norm(rb, rr) / (vravg + 0.00001)
    rbp[0] = max(rbp[0], -rl[0])
    rbp[1] = sat(rbp[1], rl[1])
    offset = -0.015 * unit(angl(rg-rbp))#+ 0.015 * unit(angl(rg-rb) + np.pi/2)

    target = rbp + offset
    
    # Ângulo da bola até o gol
    if abs(rbp[1]) >= rl[1]: angle = 0
    else: angle = ang(target, rg)

    return np.array([*target[:2], angle])