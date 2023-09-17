from strategy.BaseStrategy import Strategy
from controller import PID_control, PID_control_2, TwoSidesLQR, PID_control_3
from entities import plays
from algorithms.potential_fields import fields
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook, OnInsideBox
from entities.plays.playbook import MissBall, OnBall, AttackPossible, OnPoint,OnInsideCircle, StaticInWall, WaitFor, OnBallandWait, LeftWall
import numpy as np
class Push(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
        self.fsize = self.match.game.field.get_dimensions()
        self.delta = 0.25
    def get_name(self):
        return f"<{self.robot.get_name()} Push>"
    def start_up(self):
            super().start_up()
            controller = PID_control_3
            controller_kwargs = {
                'max_speed': 2.8, 'max_angular': 3000,'two_sides':True, 'kd': 0,  
                'kp': 30,'kb':0, 'krho': 10,'reduce_speed': False
            }

            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
    def start(self):
        pass
    def update(self):
        v = np.sqrt(self.match.ball.vx**2 + self.match.ball.vy**2)
        self.robot.strategy.controller.max_speed = 1 + v - 3*abs(self.robot.theta - np.arctan2((-self.match.ball.y + self.fsize[1]/2),(self.fsize[0]-self.match.ball.x)))
        if self.match.ball.vx != 0: 
            thetha = np.arctan2(self.match.ball.vy,self.match.ball.vx)
        else: 
            thetha = 0
        self.robot.strategy.controller.desired_angle = thetha
        ytarget = max(self.fsize[1]/2-0.3,min(self.fsize[1]/2 + 0.3,self.robot.y))
        if ((self.robot.x - self.match.ball.x)**2 + (self.robot.y - self.match.ball.y)**2)**(1/2) < self.delta:
            return [self.fsize[0],ytarget]
        else:
            return [self.match.ball.x,self.match.ball.y]
class SpinPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.fsize = self.match.game.field.get_dimensions()
        self.robot = robot
    def get_name(self):
        return f"<{self.robot.get_name()} Spin>"
    def start_up(self):
            super().start_up()
            controller = PID_control_3
            controller_kwargs = {
                'max_speed': 2.8, 'max_angular': 3000,'two_sides':True, 'kd': 0,  
                'kp': 30,'kb':0, 'krho': 10,'reduce_speed': False
            }

            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
    def start(self):
        pass
    def update(self):
        if self.robot.y > self.fsize[1]/2:
            self.robot.strategy.spin = 1000
        else:
            self.robot.strategy.spin = -1000
        return [0,0]
class Retreat(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.fsize = self.match.game.field.get_dimensions()
        self.robot = robot
    def get_name(self):
        return f"<{self.robot.get_name()} Retreat>"
    def start_up(self):
            super().start_up()
            controller = PID_control_3
            controller_kwargs = {
                'max_speed': 1.5, 'max_angular': 3000,'two_sides':True, 'kd': 0,  
                'kp': 75,'kb':0, 'krho': 10,'reduce_speed': False
            }

            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
    def start(self):
        pass
    def update(self):
        u = self.robot.x - 0
        v = self.robot.y - self.fsize[1]/2
        mod = np.sqrt(u**2+v**2)
        return [self.robot.x + u/mod,self.robot.y + v/mod]
class AjustAngle(PlayerPlay):
    def __init__(self, match, robot, nextplay, nextplay2):
        super().__init__(match, robot)
        self.fsize = self.match.game.field.get_dimensions()
        self.robot = robot
        self.nextplay = nextplay
        self.nextplay2 = nextplay2
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
        res = [self.fsize[0], self.fsize[1]/2]
        dif = self.robot.theta - np.arctan((self.robot.y-res[1])/(self.robot.x-res[0]))
        if self.robot.team_color == "yellow":
            dif -= np.pi
        delta = 0.05
        if (abs(dif) < delta):
            if ((self.robot.x - self.match.ball.x)**2 + (self.robot.y - self.match.ball.y)**2)**(1/2) < 0.4:
                self.robot.strategy.playerbook.set_play(self.nextplay)
            else:
                self.robot.strategy.playerbook.set_play(self.nextplay2)
        self.robot.strategy.spin = dif*20
        return res
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
                'max_speed': 2.3, 'max_angular': 3000,'two_sides':True, 'kd': 0,  
                'kp': 80,'kb':-30, 'krho': 13,'reduce_speed': False
            }

            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
   # def update(self):
    #    return super().update()
    def start(self):
        pass
    def update(self):
        dx = self.robot.x - self.match.ball.x
        dy = self.robot.y - self.match.ball.y

        def ajust_pred(pos):
            new_pred = pos
            if pos[1] > self.fsize[1] :
                new_pred[1] = self.fsize[1] - abs(self.fsize[1]-pos[1])/2
            elif pos[1] < 0 :
                new_pred[1] = -pos[1]/2
            #if pos[0] > self.fsize[0] :
            #    new_pred[0] = self.fsize[0] - abs(self.fsize[0]-pos[0])/2
            #elif pos[0] < 0 :
            #    new_pred[0] = -pos[0]/2
            #if new_pred[0] - self.match.ball.x < 0:
            #    new_pred[0] = self.match.ball.x
            return new_pred
        res = [0,0]
        d = ((dx)**2+(dy)**2)**(1/2)
        #self.robot.strategy.controller.v_max = 2#+ (np.pi - self.robot.strategy.controller.beta)/6
        #self.robot.strategy.controller.Ki = 10
        #self.robot.strategy.controller.KB = -40
        #self.robot.strategy.controller.KP = 70
        self.robot.reduce_speed = True
        self.robot.max_angular = 5000
        v = (self.robot.vx**2 + self.robot.vy**2)**(1/2)
        k = 0.5
        res[0] = self.match.ball.x + self.match.ball.vx*d/max(v,0.1)*k
        res[1] = self.match.ball.y + self.match.ball.vy*d/max(v,0.1)*k
        ytarget = max(self.fsize[1]/2-0.2,min(self.fsize[1]/2 + 0.2,self.robot.y))
        self.thetha = np.arctan2((-self.match.ball.y + ytarget),(self.fsize[0]-self.match.ball.x))
        if not abs(self.match.ball.y - self.fsize[1]/2) < 0.7:
            print("test")
            self.theta = 0
        self.robot.strategy.controller.desired_angle = self.thetha
        return ajust_pred(res)

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
        self.mb = MissBall(self.match, self.robot)
        self.onball = OnBall(self.match,self.robot,0.1)
        self.attack = AttackPossible(self.match,self.robot)
        self.wait = WaitFor(3)
        self.wait2 = WaitFor(0.3)
        self.wall = StaticInWall(self.match,self.robot)
        self.onb = OnBallandWait(self.match,self.robot,0.15,0.7)
        self.leftwall = LeftWall(self.match,self.robot,0.15,0.1)
        onarea = OnInsideCircle(self.robot,[0,self.fsize[1]/2],0.6)


        pred = PredictBot(self.match, self.robot)
        pred.start()

        sp = SpinPlay(self.match,self.robot)
        sp.start()

        ph = Push(self.match, self.robot)
        ph.start()

        retreat = Retreat(self.match, self.robot)

        #ajusta = AjustAngle(self.match, self.robot, push, pred)
        #ajusta.start()

        self.playerbook.add_play(pred)
        self.playerbook.add_play(ph)
        self.playerbook.add_play(sp)
        self.playerbook.add_play(retreat)

        pred.add_transition(self.onb,ph)
        pred.add_transition(self.leftwall,sp)
        sp.add_transition(self.wait2,pred)
        ph.add_transition(self.wait,pred)
        pred.add_transition(onarea,retreat)
        retreat.add_transition(self.wait,pred)
        #push.add_transition(self.attack,pred)
        self.playerbook.set_play(sp)
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