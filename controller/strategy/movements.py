from controller.tools import ang, angl, unit, angError, norm, norml, sat, shift
import numpy as np

def howFrontBall(rb, rr, rg):
    return np.dot(rr[:2]-rb, unit(angl(rg-rb)))

def howPerpBall(rb, rr, rg):
    return np.dot(rr[:2]-rb, unit(angl(rg-rb)+np.pi/2))

def projectBall(rb, vb, rr, rg, limits: tuple, vrref=0.3):
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
    offset = -0.06 * unit(angl(rg-rbp))

    return rbp + offset

def goToBall(rbpo, rg):
    # Ângulo da bola até o gol
    angle = ang(rbpo, rg)

    return np.array([*rbpo[:2], angle])

def goToGoal(rb, rg, rr):
    # Ponto de destino é a posição do gol com o ângulo do robô até o gol
    return np.array([*rg[:2], ang(rr, rg)])