from abc import ABC, abstractmethod, abstractproperty

class Environment(ABC):
    def __init__(self):
        ABC.__init__(self)
    
    @abstractmethod
    def execute(self, action):
        pass
    
    @abstractproperty
    def state(self):
        pass

    @abstractmethod
    def setInitialConditions(self):
        pass