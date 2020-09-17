import time

class Interval:
    def __init__(self):
        self.dt = None
        self.lastTime = None

    def getInterval(self):
        if self.lastTime is None:
            self.lastTime = time.time()
        else:
            currentTime = time.time()
            dt = currentTime - self.lastTime
            self.dt = dt if self.dt is None else (0.999 * self.dt + 0.001 * dt)
        
        return self.dt