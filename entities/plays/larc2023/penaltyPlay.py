import strategy
import os

from entities.plays.larc2023 import MainPlay
from strategy.larc2023.shortShooter import shortShooter
from strategy.larc2023.longShooter import longShooter

class PenaltyPlay(MainPlay):
    def __init__(self, coach, penalty_taker):
        super().__init__(coach)
        self.coach = coach
        self.match = self.coach.match

        self.penalty_taker = os.environ.get('PENALTY_TAKER', penalty_taker) 
        self.PENALTY_TAKER_ID = 4

        if self.penalty_taker == 'long_shot':
            self.shooter = longShooter(self.match, robot_id=self.PENALTY_TAKER_ID)
        elif self.penalty_taker == 'short_shot':
            self.shooter = shortShooter(self.match, robot_id=self.PENALTY_TAKER_ID)

        self.strategies = [
            strategy.tests.Idle(self.match),
            strategy.tests.Idle(self.match),
            strategy.tests.Idle(self.match),
            strategy.tests.Idle(self.match),
            self.shooter,
        ]

    def get_positions(self, foul, team_color, foul_color, quadrant):

        replacements = self.coach._get_positions(foul, team_color, foul_color, quadrant)

        if foul == "PENALTY_KICK" and foul_color == team_color:
            kicker_pos = list(filter(lambda r: r["robot_id"] == self.PENALTY_TAKER_ID, replacements))
            if len(kicker_pos):
                replacements.remove(kicker_pos[0])
            
            replacements.append(
                self.shooter.get_position()
            )
            
            return replacements

        return None

    def _can_play(self):
        return self.match.game.referee.can_play()

    def update(self):
        super().update()
