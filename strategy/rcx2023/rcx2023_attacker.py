from strategy.BaseStrategy import Strategy
from controller import PID_control, PID_control_2
from entities import plays
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook
import numpy as np

class PredictBot(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
    def get_name(self):
        return f"<{self.robot.get_name()} PredictBot>"
    def start_up(self):
            super().start_up()
            controller = PID_control_2
            controller_kwargs = {
                'max_speed': 2,'smooth_w':300, 'max_angular': 5000,'tf':True, 'kd': 0,  
                'kp': 80,'KB':-40, 'krho': 9,'reduce_speed': False
            }
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
   # def update(self):
    #    return super().update()
    def start(self):
        pass
    def update(self):
        res = [0,0]
        
        if ((self.robot.x - self.match.ball.x)**2 + (self.robot.y - self.match.ball.y)**2)**(1/2) < 0.1:
            #self.robot.strategy.controller.KB = -10
            #print("perto")
            self.robot.strategy.controller.KP = 80
            self.robot.strategy.controller.v_max = 5
            res[0] = 1.5
            res[1] = 0.65
        else:
            self.robot.strategy.controller.v_max = 2.1
            self.robot.strategy.controller.Ki = 0
            self.robot.strategy.controller.KB = -40
            self.robot.strategy.controller.KP = 30
            d = ((self.robot.x-self.match.ball.x)**2+(self.robot.y-self.match.ball.y)**2)**(1/2)
            v = (self.robot.vx**2 + self.robot.vy**2)**(1/2)
            k = 1.2
            print(self.match.ball.x,"  ", self.match.ball.y)
            res[0] = self.match.ball.x + self.match.ball.vx*d/max(v,0.1)*k
            res[1] = self.match.ball.y + self.match.ball.vy*d/max(v,0.1)*k
            print(res)
        #thetha = np.arctan2((-self.match.ball.y + 0.65),(self.match.ball.x))
        thetha = np.pi
        self.robot.strategy.controller.set_angle(thetha)
        return res


class MainAttacker(Strategy):
    def __init__(self, match, name="MainAttacker"):
        super().__init__(match, name, controller=PID_control_2)
        self.playerbook = None
    def start(self, robot=None):
        super().start(robot=robot)
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        pred = PredictBot(self.match, self.robot)
        pred.start()
        self.playerbook.add_play(pred)
        self.playerbook.set_play(pred)
    def decide(self):
        res = self.playerbook.update()
        return res
    def update(self):
        return self.controller.update()