from entities.coach.coach import BaseCoach

import strategy

class Coach(BaseCoach): # heranca da classe abstrata
    NAME = "TEST"
    def __init__(self, match, coach_parameters={}):
        super().__init__(match) # chamada do metodo da classe mae

        # vamos usar strategias de teste por enquanto, idle deixa o robo parado
        self.attacker_strategy = strategy.tests.Idle(self.match)
        self.midfielder_strategy = strategy.tests.AstarMidfielder(self.match, 'AstarMidfielder')
        self.goalkeeper_strategy = strategy.tests.Idle(self.match)
        self.radial_defender_strategy = strategy.tests.Idle(self.match)

    def decide(self):
        # esta lista eh ordenada em [robot_0, ..., robot_n]
        robots = [r.robot_id for r in self.match.robots]
        strategies = [
        self.goalkeeper_strategy,
        self.attacker_strategy, self.midfielder_strategy,
        self.radial_defender_strategy, self.radial_defender_strategy
        ]
        for robot, strategy in zip(robots, strategies):
            if self.match.robots[robot].strategy is not None:
            # vamos evitar chamar o start todo frame
            # pode ser que essa strategia seja pesada de carregar
                continue
            self.match.robots[robot].strategy = strategy
            self.match.robots[robot].start()
