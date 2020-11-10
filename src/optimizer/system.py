from abc import ABC, abstractmethod

class System(ABC):
    def __init__(self):
        ABC.__init__(self)
    
    @abstractmethod
    def action(self, state):
        pass
    
    @abstractmethod
    def metrics(self):
        pass

    @staticmethod
    def getParams(self):
        pass

    @abstractmethod
    def cost(self, experimentMetrics):
        pass

    @abstractmethod
    def endExperiment(self):
        pass