from strategy.BaseStrategy import Strategy
from controller import PID_control, PID_control_2, TwoSidesLQR
from entities import plays
from algorithms.potential_fields import fields
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook
from entities.plays.playbook import MissBall, OnBall, AttackPossible, OnPoint, StaticInWall
import numpy as np
class Push(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
    def get_name(self):
        return f"<{self.robot.get_name()} PushBot>"
    def start_up(self):
            super().start_up()
            controller = PID_control_2
            controller_kwargs = {
                'max_speed': 5,'smooth_w':300, 'max_angular': 5000,'tf':True, 'kd': 0,  
                'kp': 100,'KB':0, 'krho': 9,'reduce_speed': False, 'spread': 3/2
            }
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
    def start(self):
        pass
    def update(self):
        res = [1.5,0.65]
        return res
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
        res = [self.match.ball.x, self.match.ball.y]
        dif = self.robot.theta - np.arctan((self.robot.y-res[1])/(self.robot.x-res[0]))
        if self.robot.team_color == "yellow":
            dif -= np.pi
        delta = 0.2
        if (abs(dif) < delta):
            self.robot.strategy.playerbook.set_play(self.nextplay)
        self.robot.strategy.spin = dif*20
        return res
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
                'max_speed': 5,'smooth_w':200, 'max_angular': 5000,'tf':False, 'kd': 0,  
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
            thetha = np.arctan2((-self.match.ball.y + 0.65),(1.6-self.match.ball.x))
        else:
            self.robot.strategy.controller.smooth_w = 0
            self.robot.strategy.controller.v_max = 10
            self.robot.strategy.controller.KP = 100
            self.robot.strategy.controller.KB = 0
            self.robot.strategy.controller.KD = 0
            return [1.5,0.65]
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

    
class RecoverBall(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
    def get_name(self):
        return f"<{self.robot.get_name()} RecoverBot>"
    def start_up(self):
        super().start_up()
        controller = TwoSidesLQR
        self.robot.strategy.controller = controller(self.robot)
    def start(self):
        self.recover = fields.PotentialField(
        self.match,
        name="RecoverBehaviour"
        )
        d = ((self.robot.x-self.match.ball.x)**2+(self.robot.y-self.match.ball.y)**2)**(1/2)
        v = (self.robot.vx**2 + self.robot.vy**2)**(1/2)
        k = 1.2
        self.recover.add_field(
            fields.PointField(
                self.match,
                target = lambda m: (max(0,m.ball.x - 0.2 + m.ball.vx*d/max(v,0.1)*k), m.ball.y - 0.2*np.sin(np.arctan2((-m.ball.y + 0.65),(1.6-m.ball.x))) + m.ball.vy*d/max(v,0.1)),
                radius = 0.1,
                multiplier = 2,
                decay = lambda x : 1
            )
        )
        self.recover.add_field(
            fields.PointField(
                self.match,
                target = lambda m: (m.ball.x + 0.1, m.ball.y),
                radius = 0.01,
                multiplier = 1.5,
                decay = lambda x : -x**2
            )
        )
        #self.recover.add_field(
        #    fields.TangentialField(
        #       self.match,
        #        target = lambda m: (m.ball.x + m.ball.vx*d/max(v,0.1)*k+0.05, m.ball.y + m.ball.vy*d/max(v,0.1)-0.2),
        #        radius = 0.30,
        #        radius_max = 1,
        #        clockwise = True,
        #        decay=lambda x: -x**2,
        #        field_limits = [0.75* 2 , 0.65*2],
        #        multiplier = 0.5
        #    )
        #)
        #self.recover.add_field(
        #    fields.TangentialField(
        #        self.match,
        #        target = lambda m: (m.ball.x + m.ball.vx*d/max(v,0.1)*k+0.05, m.ball.y + m.ball.vy*d/max(v,0.1)+0.2),
        #        radius = 0.30,
        #        radius_max = 1,
        #        clockwise = False,
        #        decay=lambda x: -x**2,
        #        field_limits = [0.75* 2 , 0.65*2],
        #        multiplier = 0.5
        #    )
        #)

        for robot in self.match.robots + self.match.opposites:
            if robot.get_name() == self.robot.get_name():
                continue
            self.recover.add_field(
                fields.PointField(
                    self.match,
                    target = lambda m, r=robot: (
                        r.x,
                        r.y
                    ),
                    radius = .01,
                    decay = lambda x: -x,
                    multiplier = 0.1
                )
            )
        self.recover.add_field(
            fields.LineField(
                self.match,
                target= [self.match.game.field.get_dimensions()[0]/2, 0],                                                                                                                                                                                                                                                                                                                                          
                theta = 0,
                line_size = (self.match.game.field.get_dimensions()[0]),
                line_dist = 0.25,
                line_dist_max = 0.5,
                inverse = True,
                decay = lambda x: -x,
                multiplier = 0.3
            )
        )
        self.recover.add_field(
            fields.LineField(
                self.match,
                target= [self.match.game.field.get_dimensions()[0]/2, self.match.game.field.get_dimensions()[1]],                                                                                                                                                                                                                                                                                                                                          
                theta = 0,
                line_size = (self.match.game.field.get_dimensions()[0]),
                line_dist = 0.25,
                line_dist_max = 0.5,
                inverse = True,
                decay = lambda x: -x,
                multiplier = 0.3
            )
        )
        pass
    def update(self):
        #v = 1
        #print(self.recover.compute([self.robot.x, self.robot.y]))
        #vx = self.recover.compute([self.robot.x, self.robot.y])[0]/(self.recover.compute([self.robot.x, self.robot.y])[0]**2 + self.recover.compute([self.robot.x, self.robot.y])[1]**2)**(1/2)
        #vy = self.recover.compute([self.robot.x, self.robot.y])[1]/(self.recover.compute([self.robot.x, self.robot.y])[0]**2 + self.recover.compute([self.robot.x, self.robot.y])[1]**2)**(1/2)
        #return [vx*v, vy*v]
        return self.recover.compute([self.robot.x,self.robot.y])
class MainAttacker(Strategy):
    def __init__(self, match, name="MainAttacker"):
        super().__init__(match, name, controller=PID_control_2)
        self.playerbook = None
    def start(self, robot=None):
        super().start(robot=robot)
        self.spin = 0
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)
        self.mb = MissBall(self.match, self.robot)
        self.onball = OnBall(self.match,self.robot,0.1)
        self.attack = AttackPossible(self.match,self.robot)
        posx = lambda m: m.ball.x - 0.3*np.cos(np.arctan2((-m.ball.y + 0.65),(1.6-m.ball.x)))
        posy = lambda m: m.ball.y + 0.3*np.sin(-np.arctan2((-m.ball.y + 0.65),(1.6-m.ball.x)))
        self.op = OnPoint(self.match,self.robot,[posx,posy],0.05)
        self.wall = StaticInWall(self.match,self.robot)

        pred = PredictBot(self.match, self.robot)
        pred.start()

        recb = RecoverBall(self.match, self.robot)
        recb.start()

        push = Push(self.match, self.robot)
        push.start()
        
        ajusta = AjustAngle(self.match, self.robot, pred)
        ajusta.start()

        self.playerbook.add_play(pred)
        self.playerbook.add_play(recb)
        self.playerbook.add_play(push)
        self.playerbook.add_play(ajusta)


        pred.add_transition(self.mb,recb)
        pred.add_transition(self.wall,ajusta)
        recb.add_transition(self.onball,pred)
        recb.add_transition(self.attack,pred)
        recb.add_transition(self.op,ajusta)

        #push.add_transition(self.attack,pred)
        self.playerbook.set_play(recb)
    def decide(self):
        self.spin = 0
        res = self.playerbook.update()
        return res
    def update(self):
        if self.spin != 0:
            return self.spin, -self.spin
        else:
            return self.controller.update()