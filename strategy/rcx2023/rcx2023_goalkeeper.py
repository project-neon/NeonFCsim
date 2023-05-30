from strategy.BaseStrategy import Strategy
from controller import PID_control, PID_control_2
from entities import plays
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook
import numpy as np

class PredictKeeper(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
    def get_name(self):
        return f"<{self.robot.get_name()} PredictKeeper>"
    def start_up(self):
            super().start_up()
            controller = PID_control_2
            controller_kwargs = {
                'max_speed': 1.5,'smooth_w':300, 'max_angular': 9000,'tf':True, 'kd': 0,  
                'kp': 100,'KB':0, 'krho': 1,'reduce_speed': True
            }
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
   # def update(self):
    #    return super().update()
    def start(self):
        pass
    def update(self):
        def ajust_pred(pos):
            new_pred = pos
            if pos[1] > 1.3 :
                new_pred[1] = 1.3 - abs(1.3-pos[1])/2
            elif pos[1] < 0 :
                new_pred[1] = -pos[1]/2
            if pos[0] > 1.5 :
                new_pred[0] = 1.5 - abs(1.5-pos[0])/2
            elif pos[0] < 0 :
                new_pred[0] = -pos[0]/2
            if new_pred[0] - self.match.ball.x < 0:
                new_pred[0] = self.match.ball.x
            return new_pred
        res = [0,0]

        d = ((self.robot.x-self.match.ball.x)**2+(self.robot.y-self.match.ball.y)**2)**(1/2)
        v = (self.robot.vx**2 + self.robot.vy**2)**(1/2)
        k = 0.8
        res[0] = 0.1
        res[1] = min(max(self.match.ball.y + self.match.ball.vy*d/max(v,0.1)*k,0.3),1)
        if abs(min(max(self.match.ball.y + self.match.ball.vy*d/max(v,0.1)*k,0.3),1) - self.robot.y) < 0.06:
            self.robot.strategy.controller.max_angular = 0
            self.robot.strategy.controller.v_max = 0
        else:
            self.robot.strategy.controller.max_angular = 3000
            self.robot.strategy.controller.v_max = 1.5
        if self.match.ball.x < 0.12:
            res = [self.match.ball.x,self.match.ball.y]
            self.robot.strategy.controller.v_max = 4
            self.robot.strategy.controller.max_angular = 0
        #thetha = np.arctan2((-self.match.ball.y + 0.65),(1.6-self.match.ball.x))
        #if self.robot.strategy.controller.right:
        #    thetha += np.pi
        thetha = np.pi
        self.robot.strategy.controller.set_angle(thetha)
        return [0.1,res[1]]


class MainGoalkeeper(Strategy):
    def __init__(self, match, name="MainGoalkeeper"):
        super().__init__(match, name, controller=PID_control)
        self.playerbook = None
    def start(self, robot=None):
        super().start(robot=robot)
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        pred = PredictKeeper(self.match, self.robot)
        pred.start()
        self.playerbook.add_play(pred)
        self.playerbook.set_play(pred)
    def decide(self):
        res = self.playerbook.update()
        return res
    def update(self):
        return self.controller.update()