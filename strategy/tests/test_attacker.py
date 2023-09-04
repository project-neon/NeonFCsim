from strategy.BaseStrategy import Strategy
from controller import PID_control, PID_control_2, TwoSidesLQR, PID_control_3
from entities import plays
from algorithms.potential_fields import fields
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook
from entities.plays.playbook import MissBall, OnBall, AttackPossible, OnPoint, StaticInWall
import numpy as np


class PredictBot(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
        self.fsize = self.match.game.field.get_dimensions()
    def get_name(self):
        return f"<{self.robot.get_name()} PredictBot>"
    def start_up(self):
            super().start_up()
            controller = PID_control_3
            controller_kwargs = {
                'max_speed': 5, 'max_angular': 4000, 'kd': 0, 'two_sides': True,  
                'kp': 80,'kb':-60, 'krho': 16,'reduce_speed': True
            }

            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
   # def update(self):
    #    return super().update()
    def start(self):
        pass
    def update(self):
        res = [self.match.ball.x,self.match.ball.y]
        self.robot.strategy.controller.v_max = 2
        self.robot.strategy.controller.KP = 50
        self.robot.strategy.controller.KB = -20
        self.robot.strategy.controller.KD = 0
        self.robot.strategy.controller.desired_angle = np.pi
        return res


class MainAttacker(Strategy):
    def __init__(self, match, name="MainAttacker"):
        super().__init__(match, name, controller=PID_control_2)
        self.playerbook = None
    def start(self, robot=None):
        super().start(robot=robot)
        self.fsize = self.match.game.field.get_dimensions()
        self.spin = 0
        self.push = 0
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        pred = PredictBot(self.match, self.robot)


        self.playerbook.add_play(pred)

        #push.add_transition(self.attack,pred)
        self.playerbook.set_play(pred)
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