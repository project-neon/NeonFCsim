from strategy.BaseStrategy import Strategy
from controller import PID_control, PID_control_2, TwoSidesLQR
from entities import plays
from algorithms.potential_fields import fields
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook
from entities.plays.playbook import MissBall, OnBall, AttackPossible, OnPoint, StaticInWall
import numpy as np

class PredictBot(PlayerPlay):
    def __init__(self, match, robot, nextplay):
        super().__init__(match, robot)
        self.time = 6
        self.robot = robot
        self.fsize = self.match.game.field.get_dimensions()
        self.nextplay = nextplay
    def get_name(self):
        return f"<{self.robot.get_name()} PredictBot>"
    def start_up(self):
            super().start_up()
            controller = PID_control_2
            controller_kwargs = {
                'max_speed': 5,'smooth_w':200, 'max_angular': 5000,'tf':True, 'kd': 0,  
                'kp': 100,'KB':-80, 'krho': 9,'reduce_speed': False, 'spread': 3/2
            }

            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
   # def update(self):
    #    return super().update()
    def start(self):
        pass
    def update(self):

        if self.robot.strategy.playerbook.get_actual_play().get_running_time() > self.time:
            self.robot.strategy.playerbook.set_play(self.nextplay)
            return [0,0]

        dx = self.robot.x - self.match.ball.x
        dy = self.robot.y - self.match.ball.y
        delta = 0.08
        if np.sqrt(dx**2 + dy**2) < delta:
            if self.robot.team_color == "blue":
                self.robot.strategy.push = 20000
            else:
                self.robot.strategy.push = -20000

        def ajust_pred(pos):
            new_pred = pos
            if pos[1] > self.fsize[1] :
                new_pred[1] = self.fsize[1] - abs(self.fsize[1]-pos[1])/2
            elif pos[1] < 0 :
                new_pred[1] = -pos[1]/2
            if pos[0] > self.fsize[0] :
                new_pred[0] = self.fsize[0] - abs(self.fsize[0]-pos[0])/2
            elif pos[0] < 0 :
                new_pred[0] = -pos[0]/2
            if new_pred[0] - self.match.ball.x < 0:
                new_pred[0] = self.match.ball.x
            return new_pred
        res = [0,0]
        d = ((self.robot.x-self.match.ball.x)**2+(self.robot.y-self.match.ball.y)**2)**(1/2)
        if d > 0.15:
            self.robot.strategy.controller.v_max = 2.1 + d**2 #+ (np.pi - self.robot.strategy.controller.beta)/6
            self.robot.strategy.controller.Ki = 0
            self.robot.strategy.controller.KB = -80
            self.robot.strategy.controller.KP = 80
            v = (self.robot.vx**2 + self.robot.vy**2)**(1/2)
            k = 1.2
            res[0] = self.match.ball.x + self.match.ball.vx*d/max(v,0.1)*k
            res[1] = self.match.ball.y + self.match.ball.vy*d/max(v,0.1)*k
            thetha = np.arctan2((-self.match.ball.y + self.fsize[1]/2),(self.fsize[0] + 0.1 -self.match.ball.x))
        else:
            self.robot.strategy.controller.smooth_w = 0
            self.robot.strategy.controller.v_max = 10
            self.robot.strategy.controller.KP = 100
            self.robot.strategy.controller.KB = 0
            self.robot.strategy.controller.KD = 0
            return [self.fsize[0],self.fsize[1]/2]
        #if (self.robot.y > 1 or self.robot.y < 0.3) and (self.match.ball.y > 1 or self.match.ball.y < 0.3):
        #    self.robot.strategy.controller.extra =  + self.bot_wall_error(-10)+ self.top_wall_error(-10)
        #else:
        #    self.robot.strategy.controller.extra = 0
        #thetha = np.pi
        # + self.robots_error(-0.003)
        #print(self.robot.strategy.controller.extra)
        self.robot.strategy.controller.set_angle(thetha)
        return ajust_pred(res)
    def angle_adjustment(self,angle):
        """Adjust angle of the robot when objective is "behind" the robot"""
        phi = angle % np.radians(360)
        if phi > np.radians(180):
            phi = phi - np.radians(360)

        return phi

    
class WaitBot(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
    def get_name(self):
        return f"<{self.robot.get_name()} WaitBot>"
    def start_up(self):
        super().start_up()
        controller = TwoSidesLQR
        self.robot.strategy.controller = controller(self.robot)
    def start(self):
        self.wait = fields.PotentialField(
        self.match,
        name="WaitBehaviour"
        )

        self.wait.add_field(
            fields.PointField(
                self.match,
                target = lambda m: (0.4,0.5),
                radius = 0.001,
                multiplier = 4,
                decay = lambda x : 1
            )
        )
        
    def update(self):
        #v = 1
        #print(self.recover.compute([self.robot.x, self.robot.y]))
        #vx = self.recover.compute([self.robot.x, self.robot.y])[0]/(self.recover.compute([self.robot.x, self.robot.y])[0]**2 + self.recover.compute([self.robot.x, self.robot.y])[1]**2)**(1/2)
        #vy = self.recover.compute([self.robot.x, self.robot.y])[1]/(self.recover.compute([self.robot.x, self.robot.y])[0]**2 + self.recover.compute([self.robot.x, self.robot.y])[1]**2)**(1/2)
        #return [vx*v, vy*v]
        return self.wait.compute([self.robot.x,self.robot.y])

class AjustAngle(PlayerPlay):
    def __init__(self, match, robot, nextplay):
        super().__init__(match, robot)
        self.robot = robot
        self.nextplay = nextplay
        self.fsize = self.match.game.field.get_dimensions()

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
        res = [self.match.ball.x, self.match.ball.y]
        dif = self.robot.theta - np.arctan((self.robot.y-self.fsize[1]/2)/(self.robot.x-self.fsize[0]))
        if self.robot.team_color == "yellow":
            dif -= np.pi
        self.robot.strategy.spin = dif*20
        d = ((self.robot.x-self.match.ball.x)**2+(self.robot.y-self.match.ball.y)**2)**(1/2)
        k = 0.8
        posy = self.match.ball.y + self.match.ball.vy*d*k
        if abs(dif) < 0.1 and posy < np.arctan((self.robot.y-self.fsize[1]/2)/(self.robot.x-self.fsize[0]))*self.match.ball.x + self.robot.y:
            self.robot.strategy.playerbook.set_play(self.nextplay)

        return res

class MainMidFielder(Strategy):
    def __init__(self, match, name="MainMidFielder"):
        super().__init__(match, name, controller=PID_control)
        self.playerbook = None
    def start(self, robot=None):
        super().start(robot=robot)
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)
        wait = WaitBot(self.match,self.robot)
        wait.start()
        pred = PredictBot(self.match,self.robot,wait)
        pred.start()
        ajusta = AjustAngle(self.match,self.robot,pred)
        ajusta.start()
        op = OnPoint(self.match,self.robot,[lambda m: 0.4,lambda m: 0.5],0.05)

        self.playerbook.add_play(wait)
        self.playerbook.add_play(pred)
        self.playerbook.add_play(ajusta)

        wait.add_transition(op, ajusta)

        self.playerbook.set_play(wait)
        self.spin = 0
        
    def decide(self):
        self.spin = 0
        res = self.playerbook.update()
        return res
    def update(self):
        if self.spin != 0:
            return self.spin, -self.spin
        else:
            return self.controller.update()