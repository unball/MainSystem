import numpy as np
from tools import unit, angl, ang, norm, sat, howFrontBall, norml, projectLine, insideEllipse
import math

def goToBall(rb, vb, rg, rr, rl, vravg, offset=0.015):
    #rbp = rb + vb * norm(rb, rr) / (vravg + 0.00001)

    u = np.roots([norml(vb) ** 2 - (vravg+0.1)**2, 2 * np.dot(rb-rr[:2], vb), norml(rr[:2]-rb)**2])
    u = [x for x in u if x >= 0 and not(np.iscomplex(x))]

    if len(u) == 0:
        rbp = rb
    else:
        rbp = rb + min(u) * vb

    #rbp[0] = max(rbp[0], -rl[0])
    rbp[1] = sat(rbp[1], rl[1])
    offsetVector = offset * unit(angl(rg-rbp))#+ 0.015 * unit(angl(rg-rb) + np.pi/2)

    target = rbp + offsetVector
    
    # Ângulo da bola até o gol
    if abs(rbp[1]) >= rl[1]: angle = 0
    else: angle = ang(target, rg)

    return np.array([*target[:2], angle])

def goalkeep(rb, vb, rr, rg):
    xGoal = rg[0]
    #Se não acompanha o y
    ytarget = sat(rb[1],0.20)
    angle = np.pi/2 if rr[1] < ytarget else -np.pi/2
    return np.array([xGoal, ytarget, angle])

def blockBallElipse(rb, vb, rr, rm):
    a = 0.3
    b = 0.45
    e = np.array([1/a, 1/b])
    spin = 0
    
    d = norml(e*(rr[:2]-rm))
    #if np.abs(d-1) < 0.5: e = e / d

    finalTarget = rm

    vb = rb - finalTarget
    k = 1/np.sqrt(np.dot(e*vb, e*vb)) #* np.sign(vb[0])
    r = finalTarget + k *(vb)
    r_ = r-rm
    o = math.atan2(r_[1], r_[0])
    t = math.atan2(a*math.sin(o), b*math.cos(o))
    r_ort = (-a*math.sin(t), b*math.cos(t))
    r_ort_angle = math.atan2(r_ort[1], r_ort[0])
    
    if rr[1] > r[1] and r_ort_angle > 0: r_ort_angle = r_ort_angle+np.pi
    if rr[1] < r[1] and r_ort_angle < 0: r_ort_angle = r_ort_angle+np.pi

    if not insideEllipse(rb, a, b, rm) and norm(rr, rb) < 0.09:
       spin = 1 if rr[1] > rb[1] else -1
    
    return (r[0], r[1], r_ort_angle), spin
    # return (r[0], r[1], r_ort_angle)
    
def spinGoalKeeper(rb, rr, rm):
    if norm(rr, rb) < 0.09:
        spin = 1 if rr[1] > rb[1] else -1
    else:
        spin = 0

    return spin