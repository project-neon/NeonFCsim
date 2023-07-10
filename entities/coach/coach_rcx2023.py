from entities.coach.coach import BaseCoach
from entities import plays
import json

#import strategy

class Coach(BaseCoach): # heranca da classe abstrata
    NAME = "RCX2023"
    def __init__(self, match, coach_parameters={}):
        super().__init__(match) # chamada do metodo da classe mae

        # vamos usar strategies de teste por enquanto, essa deixa o robo parado
        self.coach_parameters = coach_parameters
        self.playbook = plays.Playbook(self)
        main_play = plays.rcx2023.MainPlay(self)

        

        self.playbook.add_play(main_play)
        self.playbook.set_play(main_play)
    def decide (self):
        
        self.playbook.update()

    def get_placement_file(self, placement_file=None):
        if placement_file:
            config = json.loads(open(placement_file, 'r').read())
        else:
            config = json.loads(open('foul_placements5v5.json', 'r').read())

        return config

    def get_positions(self, foul, team_color, foul_color, quadrant):
        if foul == 'PENALTY_KICK':
            replacement_file = self.get_placement_file()
            if replacement_file:
                replacements = replacement_file[team_color][foul][foul_color]
                return replacements
        
        return self._get_positions(foul, team_color, foul_color, quadrant)
