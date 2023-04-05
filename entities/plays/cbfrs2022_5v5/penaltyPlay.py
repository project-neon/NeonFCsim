import random
import math
import strategy
from entities.plays.cbfrs2022_5v5.mainPlay import MainPlay

class PenaltyPlay(MainPlay):
    def __init__(self, coach):
        super().__init__(coach)

        self.constraints = [
            (strategy.cbfrs2022_5v5.GoalKeeper(self.match), self._elect_goalkeeper),
            (strategy.larc2021.Shooter(self.match), self._elect_leftattacker),
            (strategy.cbfrs2022_5v5.LeftWing(self.match), self._elect_leftwing),
            (strategy.cbfrs2022_5v5.RightWing(self.match), self._elect_rightwing),
            (strategy.cbfrs2022_5v5.RightAttacker(self.match), self._elect_rightattacker)
        ]

    def get_positions(self, foul, team_color, foul_color, quadrant):
        angle_of_interest = 25
        dist_to_ball = 0.225
        shoot_side = random.choice([-1, 1])

        replacements = self.coach._get_positions(foul, team_color, foul_color, quadrant)

        if foul == "PENALTY_KICK" and foul_color == team_color:
            kicker_pos = list(filter(lambda r: r["robot_id"] == 4, replacements))
            if len(kicker_pos):
                replacements.remove(kicker_pos[0])
            field_size = self.match.game.field.get_dimensions()
            if team_color  == "BLUE":
                replacements.append(
                    {
                        "robot_id": 4, 
                        "x": field_size[0]/2 - 0.375 - math.cos(math.radians(angle_of_interest)) * dist_to_ball,
                        "y": shoot_side * math.sin(math.radians(angle_of_interest)) * dist_to_ball,
                        "orientation": - shoot_side * angle_of_interest
                    }
                )
            else:
                replacements.append(
                    {
                        "robot_id": 4, 
                        "x": - field_size[0]/2 + 0.375 + math.cos(math.radians(angle_of_interest)) * dist_to_ball,
                        "y": shoot_side * math.sin(math.radians(angle_of_interest)) * dist_to_ball,
                        "orientation": + shoot_side * angle_of_interest
                    }
                )
                
            return replacements

        return None

    def freeze_positions(self, constraints, robots):
        return constraints, robots

    def _can_play(self):
        return self.match.game.referee.can_play()

    def update(self):
        super().update()