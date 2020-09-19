import numpy as np
from tools import angl, unit, norml, angError, filt, sat

class Field:
    def __init__(self, nullgamma=False):
        self.Pb = (0,0)
        self.nullgamma = nullgamma

    def F(self, P):
        pass

    def dth(self, th2, th1, dt):
        if th1 is None or th2 is None: return 0
        return angError(th2, th1) / dt

    def phi(self, P: tuple, d=0.0001):
        """Calcula o ângulo \\(\\phi = \\frac{\\partial \\theta_d}{\\partial x} \\cos(\\theta) + \\frac{\\partial \\theta_d}{\\partial y} \\sin(\\theta)\\) usado para o controle"""
        P = np.array(P)
        dx = filt((self.F(P+[d,0,0])-self.F(P-[d,0,0]))/(2*d), 15)
        dy = filt((self.F(P+[0,d,0])-self.F(P-[0,d,0]))/(2*d), 15)
        return (dx*np.cos(P[2]) + dy*np.sin(P[2]))

    def gamma(self, dth, v, phi):
        return 0 if self.nullgamma else dth - v*phi


class UVF(Field):
    def __init__(self, Pb, radius=0.13, direction=0, spiral=True, Kr=0.2, Kr_single=0.001, nullgamma=False):
        super().__init__(nullgamma)
        # Pose final
        self.Pb = Pb

        # Raio da espiral
        self.r = radius

        # Direção da espiral, 0 para duas espirais
        self.direction = direction

        # Habilita espiral interna, caso contrário são retas 
        self.spiral = spiral

        # Constantes das espirais duplas
        self.Kr = Kr

        # Constante da espiral únicas
        self.Kr_single = Kr_single


    def F(self, P):
        return self.TUF(np.array(P), np.array(self.Pb))

    def TUF(self, P, Pb):
        P = P.copy()

        # Ajusta o sistema de coordenadas
        P[:2] = P[:2]-Pb[:2]
        rotMatrix = np.array([[np.cos(Pb[2]), np.sin(Pb[2])],[-np.sin(Pb[2]), np.cos(Pb[2])]])
        P[:2] = np.matmul(rotMatrix, P[:2])

        # Peso das espirais
        yl = -P[1] + self.r
        yr = +P[1] + self.r

        # Centros das espirais
        Pl = [P[0], P[1]-self.r]
        Pr = [P[0], P[1]+self.r]

        # Campo UVF
        if self.direction == 0:
            if np.abs(P[1]) <= self.r:
                return angl((yl*self.N_one(Pr, -1) + yr*self.N_one(Pl, +1)) / (2*self.r)) + Pb[2]
            elif P[1] < -self.r:
                return angl(self.N_one(Pr, -1)) + Pb[2]
            else:
                return angl(self.N_one(Pl, +1)) + Pb[2]
        elif self.direction == 1:
            return angl(self.M_one(Pl, +1, self.r, self.Kr_single)) + Pb[2]
        else:
            return angl(self.M_one(Pr, -1, self.r, self.Kr_single)) + Pb[2]

    def N_one(self, P, sign):
        return unit(self.alpha_one(P, sign, self.r, self.Kr))

    def M_one(self, P, sign, r, Kr):
        return unit(self.alpha_one(P, sign, r, Kr))

    def alpha_one(self, P, sign, r, Kr):
        if norml(P) >= r:
            return angl(P) + sign * np.pi/2 * (2 - (r+Kr) / (norml(P) + Kr))
        else:
            if self.spiral:
                return angl(P) + sign * np.pi/2 * np.sqrt(norml(P) / r)
            else:
                return 0


class DirectionalField(Field):
    def __init__(self, th, nullgamma=True):
        super().__init__(nullgamma=nullgamma)
        self.th = th

    def F(self, P):
        return self.th
    