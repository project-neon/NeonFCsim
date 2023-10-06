from strategy.BaseStrategy import Strategy
from controller import PID_control, PID_control_2
from entities import plays
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook
from entities.plays.playbook import GoalkeeperOut
import numpy as np

class Defend(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
    def get_name(self):
        return f"<{self.robot.get_name()} PenaltyInside>"
    def start_up(self):
            super().start_up()
            controller = PID_control
            controller_kwargs = {
                'max_speed': 1.5, 'max_angular': 9000, 'kd': 0,  
                'kp': 150, 'krho': 100,'reduce_speed': False
            }
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
   # def update(self):
    #    return super().update()
    def start(self):
        pass
    def update(self):
        res = [0,0]
        ball = self.match.ball

        projection_rate = 0.5 

        projection_point = ball.y + projection_rate * ball.vy

        y = min(max(projection_point, self.goal_right), self.goal_left)

        res[0], res[1] = [0.04, y]

        return res
    

class GoalkeeperPenaltyInside(Strategy):
    def __init__(self, match, name="GoalkeeperPenaltyInside"):
        super().__init__(match, name, controller=PID_control)
        self.playerbook = None
    def start(self, robot=None):
        super().start(robot=robot)

        defend  = Defend(self.match, self.robot)
        defend.start()

        self.playerbook.add_play(defend)

    def decide(self):
        self.spin = 0
        res = self.playerbook.update()
        return res
    def update(self):
        if self.spin != 0:
            return self.spin, -self.spin
        else:
            return self.controller.update()