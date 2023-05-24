from strategy.BaseStrategy import Strategy
from controller import PID_control, PID_control_2
from entities import plays
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook
import numpy as np
from algorithms.master_path import master_path
import matplotlib.pyplot as plt

class PathBot(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
        self.t = 0
    def get_name(self):
        return f"<{self.robot.get_name()} PathBot>"
    def start_up(self):
        super().start_up()
        controller = PID_control_2
        controller_kwargs = {
            'max_speed': 1.6,'smooth_w':300, 'max_angular': 4000,'tf':True, 'kd': 0,  
            'kp': 50,'KB':0, 'krho': 9,'reduce_speed': False
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
        self.pathplanning = master_path.MasterPath(stepsize=0.1,delta=0.11,goal=[self.match.ball.x ,self.match.ball.y])
        self.pathplanning.add_obstacle(master_path.MasterPath.semicircle_obstacle([self.match.ball.x,self.match.ball.y],0.3,"left"))
        self.pathplanning.add_obstacle(master_path.MasterPath.circle_obstacle([self.match.ball.x,self.match.ball.x-0.15],0.15))
        self.pathplanning.add_obstacle(master_path.MasterPath.circle_obstacle([self.match.ball.x,self.match.ball.x+0.15],0.15))
        self.pathplanning.add_obstacle(master_path.MasterPath.box_obstacle([0.75,-0.3],[1.5, 0.5]))
        self.pathplanning.add_obstacle(master_path.MasterPath.box_obstacle([0.75,1.8],[1.5, 0.5]))
        all = self.match.robots + self.match.opposites
        for r in all:
            if not r is self.robot:
                self.pathplanning.add_obstacle(master_path.MasterPath.circle_obstacle([r.x,r.y],0.2))
        
        for i in range(len(self.pathplanning.obstacles)):
            if i >= 5:
                self.pathplanning.obstacles[i].pos = [all[i-5].x,all[i-5].y]
        self.pathplanning.goal = [self.match.ball.x,self.match.ball.y]
        self.pathplanning.point = [self.robot.x,self.robot.y]
        self.pathplanning.path = self.pathplanning.generate_path()
        self.pathplanning.pathsmoothing(0.9,0.5,True,False)

    def start(self):
        pass

    def update(self):
        self.t += 1
        self.pathplanning.goal = [self.match.ball.x,self.match.ball.y]
        self.pathplanning.point = [self.robot.x,self.robot.y]
        self.pathplanning.obstacles[0].pos = [self.match.ball.x,self.match.ball.y]
        self.pathplanning.obstacles[1].pos = [self.match.ball.x,self.match.ball.y - 0.15]
        self.pathplanning.obstacles[2].pos = [self.match.ball.x,self.match.ball.y + 0.15]
        all = self.match.robots + self.match.opposites
        for i in range(len(self.pathplanning.obstacles)):
            if i >= 5:
                self.pathplanning.obstacles[i].pos = [all[i-5].x,all[i-5].y]
        if self.t % 100 == 0:
            self.pathplanning.path = self.pathplanning.merge(self.pathplanning.path, self.pathplanning.generate_path(), 0.8, 0.99)
            self.pathplanning.pathsmoothing(0.0,0.2,True,False)
        if ((self.robot.x - self.pathplanning.path[self.pathplanning.current].pos[0])**2 + (self.robot.y - self.pathplanning.path[self.pathplanning.current].pos[1])**2)**(1/2) < 0.17:
            self.pathplanning.next()
        thetha = np.pi
        self.robot.strategy.controller.set_angle(thetha)
        return self.pathplanning.path[self.pathplanning.current].pos



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
                'max_speed': 2,'smooth_w':900, 'max_angular': 5000,'tf':True, 'kd': 0,  
                'kp': 80,'KB':-40, 'krho': 9,'reduce_speed': False
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
        
        if ((self.robot.x - self.match.ball.x)**2 + (self.robot.y - self.match.ball.y)**2)**(1/2) < 0.05:
            self.robot.strategy.controller.KB = -20
            #print("perto")
            self.robot.strategy.controller.KP = 80
            self.robot.strategy.controller.v_max = 5
            res[0] = 1.5
            res[1] = 0.65
        else:
            self.robot.strategy.controller.v_max = 2.5
            self.robot.strategy.controller.Ki = 0
            self.robot.strategy.controller.KB = -50
            self.robot.strategy.controller.KP = 60
            d = ((self.robot.x-self.match.ball.x)**2+(self.robot.y-self.match.ball.y)**2)**(1/2)
            v = (self.robot.vx**2 + self.robot.vy**2)**(1/2)
            k = 1.2
            res[0] = self.match.ball.x + self.match.ball.vx*d/max(v,0.1)*k
            res[1] = self.match.ball.y + self.match.ball.vy*d/max(v,0.1)*k
        thetha = np.arctan2((-self.match.ball.y + 0.65),(self.match.ball.x))
        #thetha = np.pi
        self.robot.strategy.controller.set_angle(thetha)
        return ajust_pred(res)


class MainAttacker(Strategy):
    def __init__(self, match, name="MainAttacker"):
        super().__init__(match, name, controller=PID_control_2)
        self.playerbook = None
    def start(self, robot=None):
        super().start(robot=robot)
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        self.pred = PredictBot(self.match, self.robot)
        self.pred.start()
        self.path = PathBot(self.match, self.robot)
        self.path.start()
        self.playerbook.add_play(self.pred)
        self.playerbook.add_play(self.path)
        self.playerbook.set_play(self.path)
    def decide(self):
        dist = ((self.robot.x - self.match.ball.x)**2+(self.robot.y - self.match.ball.y)**2)**(1/2)
        if self.robot.x > self.match.ball.x + 0.2 and self.playerbook.actual_play != f"<{self.robot.get_name()} PathBot>":
            self.playerbook.set_play(self.path)
        elif (self.robot.x < self.match.ball.x - 0.3 or dist < 0.1 ) and self.playerbook.actual_play != f"<{self.robot.get_name()} PredictBot>":
            self.playerbook.set_play(self.pred)
        res = self.playerbook.update()
        return res
    def update(self):
        return self.controller.update()