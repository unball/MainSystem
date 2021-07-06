from tools import howFrontBall, norm
from copy import deepcopy
import numpy as np

class AttackerDecider:
    def __init__(self, world):
        self.world = world
        
        # Atacante atual
        self.currentAttacker = None

    def applyDecision(self, indexes):
        # Não há por que continuar, nenhum é melhor do que o atual
        if len(indexes) == 0:
            return True
        # Há um candidato melhor do que o atual, escolhe ele e termina
        if len(indexes) == 1:
            self.currentAttacker = indexes[0]
            return True
        # Há muitos cantidatos melhores que o atual, não termina o decisor
        else:
            return False

    def initialChoice(self, indexes):
        # Computa valores importantes
        rb = self.world.ball.pos

        # Escolhe o de menor distância à bola
        return [sorted(map(lambda i: (i, norm(self.world.team[i].pos, rb)), indexes), key=lambda x: x[1])[0][0]]

    def decide(self, indexes):# [0,1]
        """Decide o melhor atacante"""

        # Primeira iteração ou mudança na formação fez atacante atual sumir
        if self.currentAttacker not in indexes:
            self.applyDecision(self.initialChoice(indexes))
            return self.currentAttacker

        # Filtrar pelos vivos
        indexes = self.decideByAlive(indexes)
        if self.applyDecision(indexes): return self.currentAttacker

        # Filtrar por novos candidatos atrás da bola
        # indexes = self.decideByBehindBall(indexes)
        # if self.applyDecision(indexes): 
        #     print("behind ball")
        #     return self.currentAttacker

        # Filtra os próximos

        # Filtra pela velocidade da bola

        # Filtrar novos candidatos por distância
        indexes = self.decideByDistance(indexes)
        if self.applyDecision(indexes): return self.currentAttacker

        print("[AttackerDecider] Não deveria chegar até aqui!")
        return self.currentAttacker

    def decideByAlive(self, indexes):
        return [i for i in indexes if self.world.team[i].isAlive()]

    # def decideByBehindBall(self, indexes):
    #     # Remove o atacante atual da decisão
    #     indexes = [i for i in  indexes if i != self.currentAttacker]

    #     # Computa valores importantes
    #     rb = np.array(self.world.ball.pos)
    #     rg = np.array(self.world.field.goalPos)

    #     # Computa o quanto cada robo está atrás da bola
    #     currentBehindBall = -howFrontBall(rb, self.world.team[self.currentAttacker].pos, rg)
    #     howBehindBall = list(map(lambda i: (i, -howFrontBall(rb, self.world.team[i].pos, rg)), indexes))

    #     # Filtra pelos que estão melhores atrás da bola do que o atual
    #     betterBehindBall = [i for i,b in howBehindBall if b > 0 and ((currentBehindBall > 0 and 2 * b < currentBehindBall) or (currentBehindBall <= 0))]

    #     return betterBehindBall

    def decideByDistance(self, indexes):
        # Remove o atacante atual da decisão
        indexes = [i for i in  indexes if i != self.currentAttacker]

        # Computa valores importantes
        rb = self.world.ball.pos

        # Distância do atacante atual à bola
        cd = norm(self.world.team[self.currentAttacker].pos, rb)
        distances = list(map(lambda i: (i, norm(self.world.team[i].pos, rb)), indexes))

        # Filtra e ordena pelos melhores canditatos em relação à distância até a bola
        betterDistance = list(map(lambda x: x[0], sorted([(i,d) for i,d in distances if 2 * d < cd], key=lambda x: x[1])))

        if len(betterDistance) <= 1: return betterDistance
        # Retorna o melhor candidato se tiver muitos melhores que o atual
        elif len(betterDistance) > 1: return [betterDistance[0]]
