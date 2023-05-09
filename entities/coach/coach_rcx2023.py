from entities.coach.coach import BaseCoach
from entities import plays
import strategy

class Coach(BaseCoach): # heranca da classe abstrata
    NAME = "RCX2023"
    def __init__(self, match, coach_parameters={}):
        super().__init__(match) # chamada do metodo da classe mae

        # vamos usar strategies de teste por enquanto, essa deixa o robo parado
        self.coach_parameters = coach_parameters
        self.playbook = plays.Playbook(self)
        main_play = plays.rcx2023.MainPlay
        self.attacker_strategy = strategy.BaseStrategy.Strategy(self.match)
        self.midfielder_strategy = strategy.BaseStrategy.Strategy(self.match)
        self.goalkeeper_strategy = strategy.BaseStrategy.Strategy(self.match)

        self.playbook.add_play(main_play)

        self.playbook.set_play(main_play)
    def decide(self):
        # esta lista eh ordenada em [robot_0, ..., robot_n]
        robots = [r.robot_id for r in self.match.robots]
        strategies = [
        self.goalkeeper_strategy,
        self.attacker_strategy, self.midfielder_strategy

        ]
        for robot, strategy in zip(robots, strategies):
            if self.match.robots[robot].strategy is not None:
            # vamos evitar chamar o start todo frame
            # pode ser que essa strategia seja pesada de carregar
                continue
            self.match.robots[robot].strategy = strategy
            self.match.robots[robot].start()
