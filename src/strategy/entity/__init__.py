from abc import ABC, abstractmethod, abstractproperty

class Entity(ABC):
    def __init__(self, world, robot):
        ABC.__init__(self)
        
        self.world = world
        self.robot = robot
        self.name = ""

    @abstractproperty
    def control(self):
        pass

    @abstractmethod
    def fieldDecider(self):
        pass

    @abstractmethod
    def directionDecider(self):
        pass