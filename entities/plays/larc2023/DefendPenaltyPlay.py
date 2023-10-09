import strategy
import os

from entities.plays.larc2023 import MainPlay
from strategy.larc2023.PenaltyFoward import GoalkeeperPenaltyFoward
from strategy.larc2023.PenaltyInside import GoalkeeperPenaltyInside

class DefendPenaltyPlay(MainPlay):
    def __init__(self, coach, penalty_defender):
        super().__init__(coach)
        self.coach = coach
        self.match = self.coach.match

        self.penalty_defender = os.environ.get('PENALTY_DEFENDER', penalty_defender) 
        self.PENALTY_DEFENDER_ID = 4

        if self.penalty_defender == 'GoalkeeperPenaltyFoward':
            self.defender = GoalkeeperPenaltyFoward(self.match)
        elif self.penalty_defender == 'GoalkeeperPenaltyInside':
            self.defender = GoalkeeperPenaltyInside(self.match)

        self.strategies = [
            strategy.tests.Idle(self.match),
            strategy.tests.Idle(self.match),
            strategy.tests.Idle(self.match),
            strategy.tests.Idle(self.match),
            self.defender,
        ]

    def get_positions(self, foul, team_color, foul_color, quadrant):

        replacements = self.coach._get_positions(foul, team_color, foul_color, quadrant)

        if foul == "PENALTY_KICK" and foul_color != team_color:
            defender_pos = list(filter(lambda r: r["robot_id"] == self.PENALTY_DEFENDER_ID, replacements))
            if len(defender_pos):
                replacements.remove(defender_pos[0])
            
            replacements.append(
                self.defender.get_position()
            )
            
            return replacements

        return None

    def _can_play(self):
        return self.match.game.referee.can_play()

    def update(self):
        super().update()
