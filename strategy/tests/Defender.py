import math
import algorithms
import controller
import numpy as np
from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy
    
class Defender(Strategy):
    def __init__(self, match, side, plot_field=False):
        super().__init__(match, side+"Defender", controller=controller.TwoSidesLQR)
        self.name = side+"Defender"
    
    def start(self, robot=None):
        super().start(robot=robot)

        self.sobra = algorithms.fields.PotentialField(self.match, name="SobraBehaviour")

        self.project = algorithms.fields.PotentialField(self.match, name="ProjectBehaviour")

        self.path = algorithms.fields.PotentialField(self.match, name="PathBehaviour")
    
        self.kalm = algorithms.fields.PotentialField(self.match, name="KalmBehaviour")
    
        self.right_redeploy = algorithms.fields.PotentialField(self.match, name="RightRedeployBehaviour")
    
        self.left_redeploy = algorithms.fields.PotentialField(self.match, name="LeftRedeployBehaviour")    
        
        #small area x, y, width and height
        self.sa_x, self.sa_y, self.sa_w, self.sa_h = self.match.game.field.get_small_area("defensive")
    
        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.x = self.sa_w + 0.075
        
        #trave superior do gol
        g_hgr = (self.field_h/2)+0.2-0.0375
        sa_hgr = self.field_h/2 + self.sa_h/2
        ga_hgr = self.field_h/2 + 0.4
    
        #trave inferior do gol
        g_lwr = (self.field_h/2)-0.2+0.0375
        sa_lwr = self.field_h/2 - self.sa_h/2
        ga_lwr = self.field_h/2 - 0.4

        def side_verifier(y):
            d = 0.05

            if self.name == "LeftDefender":
                y += d
            elif self.name == "RightDefender":
                y -= d
            return y

        def follow_ball(m):
            if m.ball.y > g_hgr:
                y = side_verifier(g_hgr)
                return (self.x, y)
            elif m.ball.y < g_lwr:
                y = side_verifier(g_lwr)
                return (self.x, y)
            else:
                y = side_verifier(m.ball.y)
                return (self.x, y)

        self.project.add_field(
            algorithms.fields.LineField(
                self.match,
                target = follow_ball,
                theta = 0,
                line_size = self.field_h - self.sa_w,
                line_dist = 0.1,
                line_dist_max = 0.7,
                multiplier = 0.7,
                decay = lambda x : x
            )
        )
        
        def get_mid_value(a, b, c):
            return max(min(a,b), min(max(a,b),c))

        #retorna a posição em que o campo deve ser criado, para que a bola seja defendida
        def get_def_spot(m):
            x = self.x
            
            if m.ball.vx == 0:
                if m.ball.y > g_hgr:
                    y = side_verifier(g_hgr)
                    return (x, y)
                elif m.ball.y < g_lwr:
                    y = side_verifier(g_lwr)
                    return (x, y)
                else:
                    y = side_verifier(m.ball.y)
                    return (x, y)

            if m.ball.y > ga_hgr:
                y = side_verifier(g_hgr)
                return (x, y)
            elif m.ball.y < ga_lwr:
                y = side_verifier(g_lwr)
                return (x, y)
            else:
                if m.ball.x > 0.4:
                    gk_y = self.match.robots[0].y

                    if self.name == "RightDefender":
                        gk_inf = gk_y-0.075/2
                        m_inf = (gk_inf+g_lwr)/2
                        y = ( (m.ball.y-m_inf)/m.ball.x)*x + m_inf
                        y = get_mid_value(y, side_verifier(g_lwr), side_verifier(g_hgr))
                        return (x, y)
                    elif self.name == "LeftDefender":
                        gk_sup = gk_y+0.075/2
                        m_sup = (gk_sup+g_hgr)/2
                        y = ( (m.ball.y-m_sup)/m.ball.x)*x + m_sup
                        y = get_mid_value(y, side_verifier(g_lwr), side_verifier(g_hgr))
                        return (x, y)

                y = ( (m.ball.y-(self.field_h/2) )/m.ball.x)*x + self.field_h/2
                y = get_mid_value(side_verifier(y), side_verifier(g_lwr), side_verifier(g_hgr))
                return (x, y)

        def sobra(m):
            x = m.ball.x-0.8
            if m.ball.x < self.robot.x and m.ball.vx > 0:
                if m.ball.y > self.field_h/2:
                    y = m.ball.y - 0.3
                else:
                    y = m.ball.y + 0.3
            else:
                if m.ball.y > self.field_h/2:
                    y = self.field_h/2 + 0.55
                else:
                    y = self.field_h/2 - 0.55
                
                if self.field_h - y < 0.04:
                    y = self.field_h - 0.04
                elif y < 0.04:
                    y = 0.04

            return x,y
            
        self.path.add_field(
            algorithms.fields.LineField(
                self.match,
                target = get_def_spot,
                theta = 0,
                line_size = self.field_h - self.sa_w,
                line_dist = 0.1,
                line_dist_max = self.field_h,
                multiplier = 0.7,
                decay = lambda x : x
            )
        )
    
        #permanece no centro da área
        self.kalm.add_field(
            algorithms.fields.LineField(
                self.match,
                target = lambda m: (self.x, side_verifier(self.field_h/2)),
                theta = 0,
                line_size = self.field_h - self.sa_w,
                line_dist = 0.1,
                line_dist_max = self.field_h/2,
                decay = lambda x: x,
                multiplier = 0.7,
            )
        )
        
        self.left_redeploy.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target = (self.sa_w+0.0375, self.sa_h+self.sa_y - 0.07),
                radius = 0,
                radius_max = self.field_w,
                clockwise = False,
                decay = lambda x: 1,
                multiplier = 0.7
            )
        )

        self.right_redeploy.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target = (self.sa_w+0.0375, self.sa_y + 0.07),
                radius = 0,
                radius_max = self.field_w,
                clockwise = True,
                decay = lambda x: 1,
                multiplier = 0.7
            )
        )

        self.sobra.add_field(
            algorithms.fields.PointField(
                self.match,
                target = sobra,
                radius = 0.1,
                multiplier = 0.7,
                decay = lambda x : x**6
            )
        )

    
    def decide(self):
    
        self.theta = self.robot.theta
        self.maneuver = "yep"
        behaviour = None
        self.behaviour = None

        if self.match.ball.x < self.field_w/2 + 0.2:
            if (self.robot.x >= self.sa_w+0.01) and (self.robot.x < self.sa_w + 0.045):
                    
                    if self.match.ball.x > 0.225 and self.match.ball.x < self.field_w/2 + 0.3:
                        behaviour = self.path
                    elif self.match.ball.x <= 0.225:
                        behaviour = self.project
                    else:
                        behaviour = self.kalm
        
            else:
                if self.name == "LeftDefender":
                    behaviour = self.left_redeploy
                else:
                    behaviour = self.right_redeploy
        else:
            if self.name == "LeftDefender":
                self.maneuver = "nope"
                behaviour = self.sobra
            else:
                behaviour = self.kalm
    
        return behaviour.compute([self.robot.x, self.robot.y])

    def spin(self):
        if self.match.team_color.upper() == "BLUE":
            w = ((self.theta**2)**0.5 - 1.5708) * 20
        else:
            w = ((((self.theta**2)**0.5 - 4.71239)**2)**0.5) * 15
        return -w, w
    
    def spinning_time(self):
        if self.maneuver == "yep":
            if (self.robot.x > self.sa_w+0.01 and self.robot.x < self.sa_w + 0.04):
                if self.match.team_color.upper() == "BLUE":
                    if ((self.theta >= -1.61 and self.theta <= -1.54) or (self.theta >= 1.54 and self.theta <= 1.61)):
                        return False
                    else:
                        return True
                else:
                    theta = self.theta*180/math.pi
                    if ((theta >= 87 and theta <= 93) or (theta >= 267 and theta <= 273)):
                        return False
                    else:
                        return True
            else:
                return False
        else:
            return False

    def update(self):
        if self.spinning_time():
            return self.spin()
        return self.controller.update()