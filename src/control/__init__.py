from abc import ABC, abstractmethod
from tools import speeds2motors

class Control(ABC):
    def __init__(self, world):
        ABC.__init__(self)

        self.world = world

    @abstractmethod
    def output(self, robot):
        pass

    def actuate(self, robot):
        v, w = self.output(robot)
        return speeds2motors(v, self.world.field.side * w)
