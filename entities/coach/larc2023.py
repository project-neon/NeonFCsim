from entities.coach.coach import BaseCoach
from entities import plays
from entities.plays.playbook import OnPenaltyKick, OnPenaltyRecieve
import json

class Coach(BaseCoach):
    NAME = "LARC_2023_5V5"
    def __init__(self, match, coach_parameters={}):
        super().__init__(match)

        self.positions = json.loads(open('foul_placements5v5.json', 'r').read())
        
        self.penalty = json.loads(open('config.json', 'r').read())
        print(self.penalty["match"]["coach_parameters"]["penalty_taker"])
        self.playbook = plays.Playbook(self)

        main_play = plays.rcx2023.MainPlay(self)
        penalty_play = plays.larc2023.PenaltyPlay(self,self.penalty["match"]["coach_parameters"]["penalty_taker"])
        defendpenalty_play = plays.larc2023.DefendPenaltyPlay(self,self.penalty["match"]["coach_parameters"]["penalty_defender"])

        penalty_our_trigger = plays.playbook.OnPenaltyKick(self.match.game.referee, self.match.team_color)
        penalty_nour_trigger = plays.playbook.OnPenaltyRecieve(self.match.game.referee, self.match.team_color)

        penalty_seconds_trigger = plays.WaitForTrigger(10)

        main_play.add_transition(penalty_our_trigger, penalty_play)
        main_play.add_transition(penalty_nour_trigger, defendpenalty_play)

        penalty_play.add_transition(penalty_seconds_trigger, main_play)
        defendpenalty_play.add_transition(penalty_seconds_trigger, main_play)

        self.playbook.add_play(main_play)
        self.playbook.add_play(penalty_play)
        self.playbook.add_play(defendpenalty_play)

        self.playbook.set_play(main_play)

    def _get_positions(self, foul, team_color, foul_color, quadrant):
        quad = quadrant
        foul_type = foul
        team = self.positions.get(team_color)
        foul = team.get(foul)
        if not foul:
            return None

        if foul_type != "FREE_BALL":
            replacements = foul.get(foul_color, foul.get("POSITIONS"))
        else:
            replacements = foul.get(f"{quad}")
        
        return replacements

    
    def get_positions(self, foul, team_color, foul_color, quadrant):
        play_positioning = self.playbook.get_actual_play().get_positions(foul, team_color, foul_color, quadrant)
        if play_positioning:
            return play_positioning
        
        return self._get_positions(foul, team_color, foul_color, quadrant)

    def decide (self):
        self.playbook.update()
