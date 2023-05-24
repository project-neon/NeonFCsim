from strategy.BaseStrategy import Strategy
from controller import PID_control, PID_control_2
from entities import plays
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook
import numpy as np

class Beholder(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
    def get_name(self):
        return f"<{self.robot.get_name()} Beholder>"
    def start_up(self):
            super().start_up()
            controller = PID_control_2
            controller_kwargs = {
                'max_speed': 1,'smooth_w':300, 'max_angular': 5000,'tf':True, 'kd': 0,  
                'kp': 30,'KB':-40, 'krho': 9,'reduce_speed': False
            }
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
   # def update(self):
    #    return super().update()
    def start(self):
        pass
    def update(self):
        res = [0.65,1]
        for i in self.match.robots:
            if i.strategy.name == "MainAttacker" and i.y > 0.65:
                res = [0.65, 0.25]
                break
            else:
                res = [0.65,1]
        thetha = np.arctan2((-self.match.ball.y + 0.65),(self.match.ball.x))
        #thetha = np.pi
        self.robot.strategy.controller.set_angle(thetha)
        return res


class MainMidFielder(Strategy):
    def __init__(self, match, name="MainMidFielder"):
        super().__init__(match, name, controller=PID_control_2)
        self.playerbook = None
    def start(self, robot=None):
        super().start(robot=robot)
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        beholder = Beholder(self.match, self.robot)
        beholder.start()
        self.playerbook.add_play(beholder)
        self.playerbook.set_play(beholder)
    def decide(self):
        res = self.playerbook.update()
        return res
    def update(self):
        return self.controller.update()