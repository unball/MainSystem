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

    @abstractmethod
    def params(self):
        pass