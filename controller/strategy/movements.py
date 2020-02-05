from controller.tools import ang, angl, unit, angError, norm, norml, sat, shift
import numpy as np

def howFrontBall(rb, rr, rg):
    return np.dot(rr[:2]-rb, unit(angl(rg-rb)))

def howPerpBall(rb, rr, rg):
    return np.dot(rr[:2]-rb, unit(angl(rg-rb)+np.pi/2))

oldProjx = [0.0] * 10
oldProjy = [0.0] * 10

def projectBall(rb, vb, ab, rr, vr, rg, xmax, ymax, dt=0.035, vref=0.6):
    return rb
    global oldProjx
    global oldProjy

    if abs(rb[1]) < ymax-0.2 and abs(rb[0]) < xmax-0.2:
        #ts = np.roots([norml(vb) ** 2 - vr**2, 2 * np.dot(rb-rr[:2], vb), norml(rr[:2]-rb)**2 ])
        ts = np.roots([ np.dot(vb-vr, vb), np.dot(rb-rr[:2], 2*vb-vr), norml(rb-rr[:2])**2 ])
        ts = [x for x in ts if x >= 0]
        vrmod = max(norml(vr), 0.4)
        if len(ts) == 0: rbp = rb + norm(rb, rr) / vrmod * vb
        else: 
            t = min(ts)
            if np.iscomplex(t): rbp = rb + norm(rb, rr) / vrmod * vb
            else: 
                rbp = rb + t * vb
    else:
        rbp = rb

    projRobot = howFrontBall(rb, rr, rg)
    projProj = howFrontBall(rb, rbp, rg)

    if projRobot > 0 and projProj > 0:
        rbp = rb
    #vr = max(vr, 0.3)

    # if abs(rb[1]) < ymax-0.2 and abs(rb[0]) < xmax-0.2:
        
    #     ts = np.roots([1/4*norml(ab)**2, np.dot(vb, ab), norml(vb)**2 - vr**2 - np.dot(rr[:2], ab) + np.dot(rb, ab), 2*np.dot(rb-rr[:2],vb), norml(rr[:2]-rb)])
    #     ts = [x for x in ts if x >= 0]
    #     if len(ts) == 0:
    #         print("RUIM")
    #         t = max(np.roots([norml(vb) ** 2 - vr**2, 2 * np.dot(rb-rr[:2], vb), norml(rr[:2]-rb) ]))
    #         if np.iscomplex(t): rbp = rb + norm(rb, rr) / vr * vb
    #         else: rbp = rb + t * vb
    #     else:
    #         t = min(ts)
    #         # Projeção da bola com a velocidade
    #         if np.iscomplex(t): 
    #             print("RUIM")
    #             rbp = rb + norm(rb, rr) / vr * vb
    #         else: 
    #             print("BOM")
    #             rbp = rb + t * vb + t**2/2 * ab
    #     rbp = np.real(rbp)
    # else:
    #     rbp = rb

    rbp = (sat(rbp[0], xmax), sat(rbp[1], ymax))

    rbpnew = ((rbp[0]+sum(oldProjx))/11.0, (rbp[1]+sum(oldProjy))/11.0)
    oldProjx = shift(rbp[0], oldProjx)
    oldProjy = shift(rbp[1], oldProjy)
    rbp = rbpnew

    # Vetor no sentido da bola para o gol de deslocamento
    #f = np.exp(-(norm(rb, rr) / (0.095*2))**2) * np.exp(-((rb[0]-rr[0]) / (0.05))**2) if rr[0] < rb[0] else 0
    k = howFrontBall(rb, rr, rg)
    p = howPerpBall(rb, rr, rg)
    offset = -0.03 * unit(angl(rg-rbp)) #+ 0.06 * unit(angl(rg-rbp)) * np.exp(-norm(rb, rr)/0.21)

    return rbp + offset

def goToBall(rbpo, rg, ymax):
    # Ângulo da bola até o gol
    angle = ang(rbpo, rg)

    if abs(rbpo[1]) > 0.98 * ymax: ang((rbpo[0], 0.98 * ymax * np.sign(rbpo[1])), rg)

    return np.array([*rbpo[:2], angle])

def goToGoal(rg, rr):
    # Ponto de destino é a posição do gol com o ângulo do robô até o gol
    return np.array([*rg[:2], ang(rr, rg)])