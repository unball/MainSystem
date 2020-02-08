from controller.tools import ang, angl, unit, angError, norm, norml, sat, shift, derivative
import numpy as np

def howFrontBall(rb, rr, rg):
    return np.dot(rr[:2]-rb, unit(angl(rg-rb)))

def howPerpBall(rb, rr, rg):
    return np.dot(rr[:2]-rb, unit(angl(rg-rb)+np.pi/2))

def projectBall(rb, vb, rr, rg, limits: tuple, vrref=0.25):
    # Bola fora dos limites, retorna a própria posição da bola
    if any(np.abs(rb) > limits):
        return rb

    # Computa os tempos de interceptação possíveis
    ts = np.roots([norml(vb) ** 2 - vrref**2, 2 * np.dot(rb-rr[:2], vb), norml(rr[:2]-rb)**2 ])

    # Filtra por tempos positivos
    ts = [x for x in ts if x >= 0]

    # Não conseguiu encontrar um tempo positivo real
    if len(ts) == 0 or np.iscomplex(min(ts)):
        rbp = rb + norm(rb, rr) / vrref * vb
    
    # Computa a projeção
    else:
        t = min(ts)
        rbp = rb + t * vb

    # Acrescenta um offset
    offset = -0.06 * unit(angl(rg-rbp)) + 0.015 * unit(angl(rg-rbp) + np.pi/2)

    return rbp + offset

def goToBall(rb, rg, vb):
    # Ângulo da bola até o gol
    angle = ang(rb, rg)

    dth = derivative(lambda x : ang((x, rb[1]), rg), rb[0]) * vb[0] + derivative(lambda y : ang((rb[0], y), rg), rb[1]) * vb[1]
    v = (*vb, dth)
    return np.array([*rb[:2], angle]), v

def goToGoal(rg, rr):
    # Ponto de destino é a posição do gol com o ângulo do robô até o gol
    return np.array([*rg[:2], ang(rr, rg)]), (0,0,0)