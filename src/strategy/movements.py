import numpy as np
from tools import unit, angl, ang, norm, sat, howFrontBall, norml, projectLine, insideEllipse, howPerpBall
import math

def goToBall(rb, vb, rg, rr, rl, vravg, offset=0.015):
    rb = rb.copy()
    #rbp = rb + vb * norm(rb, rr) / (vravg + 0.00001)

    u = np.roots([norml(vb) ** 2 - (max(vravg-0.1, 0))**2, 2 * np.dot(rb-rr[:2], vb), norml(rr[:2]-rb)**2])
    u = [x for x in u if x >= 0 and not(np.iscomplex(x))]

    if len(u) == 0 or norm(rb, rr) < 0.1:
        rbp = rb
    else:
        rbp = rb + min(u) * vb
        
    #rbp = rb

    #rbp[0] = max(rbp[0], -rl[0])
    #rbp[0] = sat(rbp[0], rg[0])
    rbp[1] = sat(rbp[1], rl[1])
    offsetVector = offset * unit(angl(rg-rbp))#+ 0.015 * unit(angl(rg-rb) + np.pi/2)

    target = rbp + offsetVector
    
    # Ângulo da bola até o gol
    if abs(rbp[1]) >= rl[1]: angle = 0
    else: angle = ang(target, rg)

    return np.array([*target[:2], angle])

def goalkeep(rb, vb, rr, rg):
    xGoal = rg[0] 

    #projeta a velocidade da bola 
    ytarget = projectLine(rb, vb, xGoal+0.05)
    if ((vb[0]) < -0.05): #and  ((rb[0]) > .15) and np.abs(ytarget) < 0.2:
        #verificar se a projeção está no gol
        ytarget = sat(ytarget, 0.25)
        angle = np.pi/2 if rr[1] < ytarget else -np.pi/2
        return (xGoal, ytarget, angle)

    #Se não, acompanha o y
    ytarget = sat(rb[1],0.25)
    angle = np.pi/2 if rr[1] < ytarget else -np.pi/2
    return np.array([xGoal, ytarget, angle])
 
def blockBallElipse(rb, vb, rr, rm, a, b):
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

    # if insideEllipse(rb, a, b, rm):
    #     return (r[0], -r[1], r_ort_angle), spin
    
    return (r[0], r[1], r_ort_angle), spin
    # return (r[0], r[1], r_ort_angle)
    
def spinGoalKeeper(rb, rr, rm):
    if norm(rr, rb) < 0.08:
        spin = 1 if rr[1] > rb[1] else -1
    else:
        spin = 0

    return spin