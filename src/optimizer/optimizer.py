from abc import ABC, abstractmethod
import time

class Optimizer(ABC):
    def __init__(self, systemClass, environmentClass):
        ABC.__init__(self)
        self.systemClass = systemClass
        self.environmentClass = environmentClass

    def loop(self, system, environment):
        self.ctime = time.time()
        state  = environment.state
        action = system.action(state)
        
        environment.execute(action)

        return system.metrics

    @abstractmethod
    def optimize(self):
        pass

    def runExperiment(self, params):
        system = self.systemClass(params)
        environment = self.environmentClass()
        environment.setInitialConditions()
        
        experimentMetrics = []

        while not system.endExperiment():
            loopMetric = self.loop(system, environment)
            experimentMetrics.append(loopMetric)

        return system.cost(experimentMetrics)