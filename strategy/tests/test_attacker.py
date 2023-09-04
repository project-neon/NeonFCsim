from strategy.BaseStrategy import Strategy
from controller import PID_control, PID_control_2, TwoSidesLQR
from entities import plays
from algorithms.potential_fields import fields
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook, OnInsideBox
from entities.plays.playbook import MissBall, OnBall, AttackPossible, OnPoint, StaticInWall
import numpy as np


class BallFollower(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
        self.fsize = self.match.game.field.get_dimensions()
    def get_name(self):
        return f"<{self.robot.get_name()} BallFollower>"
    def start_up(self):
            super().start_up()
            controller = PID_control
            controller_kwargs = {
                'max_speed': 5, 'max_angular': 4000, 'kd': 0,  
                'kp': 80, 'krho': 8,'reduce_speed': True
            }

            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
   # def update(self):
    #    return super().update()
    def start(self):
        pass
    def update(self):
        res = [self.match.ball.x,self.match.ball.y]
        return res
    
class waitplay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
        self.fsize = self.match.game.field.get_dimensions()
    def get_name(self):
        return f"<{self.robot.get_name()} waitplay>"
    def start_up(self):
            super().start_up()
            controller = PID_control
            controller_kwargs = {
                'max_speed': 5, 'max_angular': 4000, 'kd': 0,
                'kp': 80, 'krho': 8,'reduce_speed': True
            }

            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
   # def update(self):
    #    return super().update()
    def start(self):
        pass
    def update(self):
        res = [0.6,0.75]
        return res

class MainAttacker(Strategy):
    def __init__(self, match, name="MainAttacker"):
        super().__init__(match, name, controller=PID_control)
        self.playerbook = None
    def start(self, robot=None):
        super().start(robot=robot)
        self.fsize = self.match.game.field.get_dimensions()
        self.spin = 0
        self.push = 0
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        insidebox1 = OnInsideBox(self.match,[0,0,self.fsize[0]/2,self.fsize[1]])
        insidebox2 = OnInsideBox(self.match,[self.fsize[0]/2,0,self.fsize[0]/2,self.fsize[1]])

        bf = BallFollower(self.match, self.robot)
        bf.start()
        
        wp = waitplay(self.match, self.robot)
        wp.start()

        self.playerbook.add_play(bf)
        self.playerbook.add_play(wp)

        bf.add_transition(insidebox1, wp)
        wp.add_transition(insidebox2, bf)

        #push.add_transition(self.attack,pred)
        self.playerbook.set_play(bf)
    def decide(self):
        self.spin = 0
        self.push = False
        res = self.playerbook.update()
        print(self.playerbook.get_actual_play())
        return res
    def update(self):
        if self.push != 0:
            return self.push, self.push
        if self.spin != 0:
            return self.spin, -self.spin
        else:
            return self.controller.update()