from controller.tools import ang, angl, unit, angError, norm, norml, sat
import numpy as np

def howFrontBall(rb, rr, rg):
    return np.dot(rr[:2]-rb, unit(angl(rg-rb)))

def howPerpBall(rb, rr, rg):
    return np.dot(rr[:2]-rb, unit(angl(rg-rb)+np.pi/2))

def projectBall(rb, vb, ab, rr, vr, rg, xmax, ymax, dt=0.035, vref=0.6):
    k = howFrontBall(rb, rr, rg) - 0.1

    if True or k > 0:
        t = max(np.roots([norml(vb) ** 2 - vr**2, 2 * np.dot(rb-rr[:2], vb), norml(rr[:2]-rb) ]))
        if np.iscomplex(t): rbp = rb + norm(rb, rr) / vr * vb
        else: rbp = rb + t * vb
    else:
        rbp = rb
    # vr = max(vr, 0.3)
    # ts = np.roots([1/4*norml(ab)**2, np.dot(vb, ab), norml(vb)**2 - vr**2 - np.dot(rr[:2], ab) + np.dot(rb, ab), 2*np.dot(rb-rr[:2],vb), norml(rr[:2]-rb)])
    # ts = [x for x in ts if x >= 0]
    # if len(ts) == 0:
    #     print("RUIM")
    #     t = max(np.roots([norml(vb) ** 2 - vr**2, 2 * np.dot(rb-rr[:2], vb), norml(rr[:2]-rb) ]))
    #     if np.iscomplex(t): rbp = rb + norm(rb, rr) / vr * vb
    #     else: rbp = rb + t * vb + t**2/2 * ab
    # else:
    #     t = min(ts)
    #     # Projeção da bola com a velocidade
    #     if np.iscomplex(t): 
    #         print("RUIM")
    #         rbp = rb + norm(rb, rr) / vr * vb
    #     else: 
    #         print("BOM")
    #         rbp = rb + t * vb + t**2/2 * ab
    #rbp = np.real(rbp)

    rbp = (sat(rbp[0], xmax), sat(rbp[1], ymax))
    # Vetor no sentido da bola para o gol de deslocamento
    #f = np.exp(-(norm(rb, rr) / (0.095*2))**2) * np.exp(-((rb[0]-rr[0]) / (0.05))**2) if rr[0] < rb[0] else 0
    k = howFrontBall(rb, rr, rg)
    p = howPerpBall(rb, rr, rg)
    offset = -0.06 * unit(angl(rg-rbp)) + 0.06 * unit(angl(rg-rbp)) * np.exp(-norm(rb, rr)/0.21)

    return rbp + offset

def goToBall(rb, vb, ab, rr, vr, rg, xmax, ymax):
    # Projeção da bola com base na velocidade + um offset
    rbpo = projectBall(rb, vb, ab, rr, vr, rg, xmax, ymax)

    # Ângulo da bola até o gol
    angle = ang(rbpo, rg)

    if abs(rbpo[1]) > 0.98 * ymax: ang((rbpo[0], 0.98 * ymax * np.sign(rbpo[1])), rg)

    return np.array([*rbpo[:2], angle])

def goToGoal(rg, rr):
    # Ponto de destino é a posição do gol com o ângulo do robô até o gol
    return np.array([*rg[:2], ang(rr, rg)])