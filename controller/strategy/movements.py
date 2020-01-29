from controller.tools import ang, angl, unit, angError, norm, norml
import numpy as np

def howFrontBall(rb, rr, rg):
    return np.dot(rr[:2]-rb, unit(angl(rg-rb)))

def projectBall(rb, vb, rr, vr, rg, dt=0.035, vref=0.4):
    t = max(np.roots([norml(vb) ** 2 - vr, 2 * np.dot(rb-rr[:2], vb), norml(rr[:2]-rb) ]))

    # Projeção da bola com a velocidade
    if np.iscomplex(t): rbp = rb + norm(rb, rr) / vr * vb
    else: rbp = rb + 0.1* t * vb    

    # Vetor no sentido da bola para o gol de deslocamento
    #f = np.exp(-(norm(rb, rr) / (0.095*2))**2) * np.exp(-((rb[0]-rr[0]) / (0.05))**2) if rr[0] < rb[0] else 0
    k = howFrontBall(rb, rr, rg) - 0.03
    offset = 0.04 * np.arctan(-k / 0.03) / (np.pi/2) * unit(angl(rg-rbp))
        
    return rbp + offset

def goToBall(rb, vb, rr, vr, rg, ymax):
    # Projeção da bola com base na velocidade + um offset
    rbpo = projectBall(rb, vb, rr, vr, rg)

    # Ângulo da bola até o gol
    angle = ang(rbpo, rg)

    if abs(rbpo[1]) > 0.98 * ymax: ang((rbpo[0], 0.98 * ymax * np.sign(rbpo[1])), rg)

    return np.array([*rbpo[:2], angle])

def goToGoal(rg, rr):
    # Ponto de destino é a posição do gol com o ângulo do robô até o gol
    return np.array([*rg[:2], ang(rr, rg)])