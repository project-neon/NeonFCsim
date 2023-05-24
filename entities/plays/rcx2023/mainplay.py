import math
import strategy
from entities.plays.playbook import Play
import time

class MainPlay(Play):
    def __init__(self, coach):
        super().__init__(coach)
        self.match = self.coach.match
        self.coach = coach
        self.strategies = [
            strategy.Strategy(self.match, 'GoalKeeper'),
            strategy.rcx2023.MainMidFielder(self.match, 'MainMidfielder'),
            strategy.rcx2023.MainAttacker(self.match, 'MainAttacker'),
        ]

    def _can_play(self):
        return self.match.game.referee.can_play()

    def start_up(self):
        self.start_running_time = time.time()
   #     super().start_up()

    def update(self):
        super().update()
        
        for i in self.match.robots:
            if i.strategy is None:
                i.strategy = self.strategies[i.robot_id]
