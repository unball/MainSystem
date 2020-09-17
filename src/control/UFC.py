from tools import speeds2motors, angError, sat
from tools.interval import Interval

class UFC:
    def __init__(self):
        self.T = 0.10
        self.Kw = 30
        self.vmax = 2
        self.mu = 1
        self.g = 9.85
        self.th = None
        self.vrmax = 1000

        self.intervalManager = Interval()
        self.lastth = None

    def actuate(self, robot):
        # Intervalo de tempo desde a última chamada
        dt = self.intervalManager.getInterval()

        # Referência
        th = robot.field.F(robot.pose)
        #print(robot.th)

        # Derivada
        dth = robot.field.dth(th, self.lastth, dt)

        # Erro
        eth = angError(th, robot.th)
        print(eth)

        # Controle
        u = dth + self.T * robot.alpha + self.Kw * eth
        v = min(self.vmax, abs(self.mu * self.g / (robot.w + 0.0001)))

        # Atualiza estados
        self.lastth = th

        vl, vr = speeds2motors(v, u)

        return sat(vl, self.vrmax), sat(vr, self.vrmax)