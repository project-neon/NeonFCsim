from entities.coach.coach import BaseCoach
from entities import plays
#import strategy

class Coach(BaseCoach): # heranca da classe abstrata
    NAME = "RCX2023"
    def __init__(self, match, coach_parameters={}):
        super().__init__(match) # chamada do metodo da classe mae

        # vamos usar strategies de teste por enquanto, essa deixa o robo parado
        self.coach_parameters = coach_parameters
        self.playbook = plays.Playbook(self)
        main_play = plays.rcx2023.MainPlay

        

        self.playbook.add_play(main_play)
        self.playbook.set_play(main_play)
    def decide(self):
        return
