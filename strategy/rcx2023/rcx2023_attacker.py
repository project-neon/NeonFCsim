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
                'max_speed': 5,'smooth_w':300, 'max_angular': 5000,'tf':True, 'kd': 0,  
                'kp': 80,'KB':-40, 'krho': 9,'reduce_speed': False, 'spread': 3/2
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
        
        if ((self.robot.x - self.match.ball.x)**2 + (self.robot.y - self.match.ball.y)**2)**(1/2) < 0.1:
            self.robot.strategy.controller.KB = -20
            #print("perto")
            self.robot.strategy.controller.KP = 20
            self.robot.strategy.controller.v_max = 5
            res[0] = 1.5
            res[1] = 0.65
            thetha = np.arctan2((-self.match.ball.y + 0.65),(1.6-self.match.ball.x))
        else:
            self.robot.strategy.controller.v_max = 2.1
            self.robot.strategy.controller.Ki = 0
            self.robot.strategy.controller.KB = -80
            self.robot.strategy.controller.KP = 80
            d = ((self.robot.x-self.match.ball.x)**2+(self.robot.y-self.match.ball.y)**2)**(1/2)
            v = (self.robot.vx**2 + self.robot.vy**2)**(1/2)
            k = 1.2
            res[0] = self.match.ball.x + self.match.ball.vx*d/max(v,0.1)*k
            res[1] = self.match.ball.y + self.match.ball.vy*d/max(v,0.1)*k
            thetha = np.arctan2((-self.match.ball.y + 0.65),(1.6-self.match.ball.x))
            #self.robot.strategy.controller.extra =  + self.bot_wall_error(20)+ self.top_wall_error(20)
            #if self.robot.x > self.match.ball.x + 0.2 and (self.robot.vx**2 + self.robot.vy**2)**(1/2) < 0.05:
            #    self.robot.strategy.controller.spread = 2/3
            #else:
            #    self.robot.strategy.controller.spread = 0
        print( self.robot.strategy.controller.right)
        #if self.robot.strategy.controller.right:
        #    thetha = np.pi/6

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
    def robots_error(self,k):
        error = 0
        for r in self.match.opposites + self.match.robots:
            if r.get_name() != self.robot.get_name():
                D_x = r.x - self.robot.x
                D_y = r.y - self.robot.y
                Dist = (D_x**2 + D_y**2)**(1/2)
                gamma = np.arctan2(D_y,D_x) + np.pi/2
                alpha = gamma - self.robot.theta 
                if self.robot.strategy.controller.right:
                    error -= self.angle_adjustment(alpha)*(1.8 - Dist)
                else:
                    error += self.angle_adjustment(alpha)*(1.8 - Dist)
        return k*error
    def top_wall_error(self,k):
        error = 0
        gamma = np.pi
        Dist = abs(self.robot.y - 1.3)
        alpha = self.angle_adjustment(gamma - self.robot.theta)
        if self.robot.strategy.controller.right:
            error -= self.angle_adjustment(alpha)*(1.3 - abs(Dist))
        else:
            error += self.angle_adjustment(alpha)*(1.3 - abs(Dist))
        return k*error
    def bot_wall_error(self,k):
        error = 0
        gamma = 0
        Dist = abs(self.robot.y)
        alpha = self.angle_adjustment(gamma - self.robot.theta)
        if self.robot.strategy.controller.right:
            error -= self.angle_adjustment(alpha)*(1.3 - abs(Dist))**5
        else:
            error += self.angle_adjustment(alpha)*(1.3 - abs(Dist))**5
        return k*error
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