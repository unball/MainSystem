from abc import ABC, abstractmethod

class Optimizer(ABC):
    def __init__(self, systemClass, environmentClass):
        ABC.__init__(self)
        self.systemClass = systemClass
        self.environmentClass = environmentClass

    def loop(self, system, environment):
        state  = environment.state
        action = system.action(state)
        
        environment.execute(action)

        return system.metrics

    @abstractmethod
    def endExperimentCondition(self):
        pass

    @abstractmethod
    def cost(self, experimentMetrics):
        pass

    @abstractmethod
    def optimize(self):
        pass

    def runExperiment(self, params):
        system = self.systemClass(params)
        environment = self.environmentClass()
        environment.setInitialConditions()
        
        experimentMetrics = []

        while not self.endExperimentCondition():
            loopMetric = self.loop(system, environment)
            experimentMetrics.append(loopMetric)

        return self.cost(experimentMetrics)