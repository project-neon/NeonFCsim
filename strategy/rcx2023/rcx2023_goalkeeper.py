from strategy.BaseStrategy import Strategy
from controller import PID_control, PID_control_2
from entities import plays
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook
from entities.plays.playbook import GoalkeeperOut
import numpy as np

class AjustAngle(PlayerPlay):
    def __init__(self, match, robot, nextplay):
        super().__init__(match, robot)
        self.robot = robot
        self.nextplay = nextplay
    def get_name(self):
        return f"<{self.robot.get_name()} AjustAngle>"
    def start_up(self):
            super().start_up()
            controller = PID_control_2
            controller_kwargs = {
                'max_speed': 0,'smooth_w':300, 'max_angular': 0,'tf':True, 'kd': 0,  
                'kp': 200,'KB':0, 'krho': 9,'reduce_speed': False, 'spread': 3/2
            }
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
    def start(self):
        pass
    def update(self):
        dif = self.robot.theta - np.pi
        if self.robot.team_color == "yellow":
            dif -= np.pi
        delta = 0.2
        if (abs(dif) < delta):
            self.robot.strategy.playerbook.set_play(self.nextplay)
        self.robot.strategy.spin = dif*20
        return [0,0]


class PredictKeeper(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
    def get_name(self):
        return f"<{self.robot.get_name()} PredictKeeper>"
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
        x = 0.07
        d = ((self.robot.x-self.match.ball.x)**2+(self.robot.y-self.match.ball.y)**2)**(1/4)
        v = (self.robot.vx**2 + self.robot.vy**2)**(1/2)
        k = 0.8
        if self.match.game.field.get_dimensions()[1] == 1.3:
            res[0] = x
            res[1] = min(max(self.match.ball.y - v*d*(-self.match.ball.x)*self.match.ball.vy/max(self.match.ball.vx,0.1),0.3),1)
        else:
            res[0] = x
            res[1] = min(max(self.match.ball.y - v*d*(-self.match.ball.x)*self.match.ball.vy/max(self.match.ball.vx,0.1),0.65),1.15)
        if abs(self.match.ball.vy) < 0.03 and abs(self.robot.y - res[1]) < 0.07:
            self.robot.strategy.controller.max_angular = 0
            self.robot.strategy.controller.v_max = 0
        else:
            self.robot.strategy.controller.max_angular = 3000
            self.robot.strategy.controller.v_max = 1 + abs(self.robot.y - res[1])
        if self.match.ball.x < x - 0.05:
            res = [self.match.ball.x,self.match.ball.y]
            self.robot.strategy.controller.v_max = 4
            self.robot.strategy.controller.max_angular = 0
        #thetha = np.arctan2((-self.match.ball.y + 0.65),(1.6-self.match.ball.x))
        #if self.robot.strategy.controller.right:
        #    thetha += np.pi
        return res
class AjustPosition(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
    def get_name(self):
        return f"<{self.robot.get_name()} AjustPosition>"
    def start_up(self):
            super().start_up()
            controller = PID_control
            controller_kwargs = {
                'max_speed': 1.5, 'max_angular': 9000, 'kd': 0,  
                'kp': 40, 'krho': 1,'reduce_speed': False
            }
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
   # def update(self):
    #    return super().update()
    def start(self):
        pass
    def update(self):
        return [0.1, self.robot.y]

class MainGoalkeeper(Strategy):
    def __init__(self, match, name="MainGoalkeeper"):
        super().__init__(match, name, controller=PID_control)
        self.playerbook = None
    def start(self, robot=None):
        super().start(robot=robot)
        self.spin = 0
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)
        out = GoalkeeperOut(self.match, self.robot, 0.2)
        ingoal = GoalkeeperOut(self.match, self.robot, 0.2, True)

        ajustp = AjustPosition(self.match, self.robot)
        ajustp.start()

        pred = PredictKeeper(self.match, self.robot)
        pred.start()

        ajusta = AjustAngle(self.match, self.robot, pred)
        ajusta.start()

        self.playerbook.add_play(pred)
        self.playerbook.add_play(ajustp)
        self.playerbook.add_play(ajusta)
        pred.add_transition(out,ajustp)
        ajustp.add_transition(ingoal,ajusta)
        self.playerbook.set_play(pred)
    def decide(self):
        self.spin = 0
        res = self.playerbook.update()
        return res
    def update(self):
        if self.spin != 0:
            return self.spin, -self.spin
        else:
            return self.controller.update()