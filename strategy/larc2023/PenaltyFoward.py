from strategy.BaseStrategy import Strategy
from controller import PID_control, PID_control_2
from entities import plays
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook
from entities.plays.playbook import GoalkeeperOut
import numpy as np

class Foward(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
    def get_name(self):
        return f"<{self.robot.get_name()} PenaltyFoward>"
    def start_up(self):
            super().start_up()
            controller = PID_control
            controller_kwargs = {
                'max_speed': 1.5, 'max_angular': 9000, 'kd': 0,  
                'kp': 150, 'krho': 100,'reduce_speed': True
            }
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
   # def update(self):
    #    return super().update()
    def start(self):
        pass
    def update(self):
        res = [0,0]
        ball = self.match.ball

        projection_ratey = 0.3

        projection_point = ball.y + projection_ratey * ball.vy

        y = min(max(projection_point, self.goal_right), self.goal_left)

        res[0], res[1] = [ball.x, y]

        return res
    

class GoalkeeperPenaltyFoward(Strategy):
    def __init__(self, match, name="GoalkeeperPenaltyFoward"):
        super().__init__(match, name, controller=PID_control)
        self.playerbook = None
    def start(self, robot=None):
        super().start(robot=robot)

        foward = Foward(self.match, self.robot)
        foward.start()

        self.playerbook.add_play(foward)
    def get_position(self):
    	pos = {}
    	if self.match.team_color == "blue":
    		pos = {"robot_id": self.robot_id, "x": -1.06, "y": 0, "orientation": 0}
    	else:
    		pos = {"robot_id": self.robot_id, "x": 1.06, "y": 0, "orientation": 0}
    	return pos 
    def decide(self):
        self.spin = 0
        res = self.playerbook.update()
        return res
    def update(self):
        if self.spin != 0:
            return self.spin, -self.spin
        else:
            return self.controller.update()
