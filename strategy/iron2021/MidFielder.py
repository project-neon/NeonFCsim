import math
import algorithms
import controller
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector, distance

import json
import numpy as np

def point_in_rect(point,rect):
    x1, y1, w, h = rect
    x2, y2 = x1+w, y1+h
    x, y = point
    if (x1 < x and x < x2):
        if (y1 < y and y < y2):
            return True
    return False

class MidFielder(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, "midfielder", controller=controller.TwoSidesLQR)

        """
        Essa estrategia descreve a um goleiro base, simples, que
        pode ser usado como baseline para o desenvolvimento de 
        outros goleiros.

        Sua estrategia é constituida por alguns contextos que são
        mudados baseados em regras simples dentro do jogo.
        Cada contexto é um campo potencial que rege que tipo de
        movimentação o goleiro precisa fazer. Sendo elas:

        1) maintain: O goleiro se mantem estatico no centro do gol

        1) alert: O goleiro se move dentro do gol, na pequena
        area, seguindo a bola caso ela esteja dentro do eixo X do gol
        e ficando nas "traves" caso a bola esteja nas laterais e dentro
        da nossa area

        2) push: O goleiro empurra a bola caso ela esteja vindo em sua
        direção, saindo um pouco da pequena area

        A troca entre os contextos reside no metodo decide()
        """
        
        self.normal_speed = 0.75
        self.obey_rules_speed = 0.5
        self.push_speed = 0.8

        self.plot_field = plot_field
        self.exporter = None

    def start(self, robot=None):
        super().start(robot=robot)

        self.base_rules = algorithms.fields.PotentialField(
            self.match,
            name="{}|BaseRulesBehaviour".format(self.__class__)
        )
        
        self.detain = algorithms.fields.PotentialField(
            self.match, 
            name="{}|DetainBehaviour".format(self.__class__)
        )

        self.wait = algorithms.fields.PotentialField(
            self.match, 
            name="{}|WaitBehaviour".format(self.__class__)
        )

        self.defend = algorithms.fields.PotentialField(
            self.match,
            name="{}|DefendBehaviour".format(self.__class__)
        )
        
        if self.plot_field:
            self.exporter = algorithms.fields.PotentialDataExporter(self.robot.get_name())

        def pl(self):
            robot_id = self.robot.robot_id
            radius = 0.2

            def s(m):
                robot_pos = [m.robots[robot_id].x, m.robots[robot_id].y]
                ball_pos = [m.ball.x, m.ball.y]
                goal_pos = [0.75, 0.65]

                dist = distance(goal_pos, ball_pos, robot_pos)

                weight = 1/2 + 1/2 * min((dist/radius), 1)

                return weight * 1 if m.ball.y < 0.65 else 0
            
            return s
        
        def pr(self):
            robot_id = self.robot.robot_id
            radius = 0.2

            def s(m):
                robot_pos = [m.robots[robot_id].x, m.robots[robot_id].y]
                ball_pos = [m.ball.x, m.ball.y]
                goal_pos = [0.75, 0.65]

                dist = distance(goal_pos, ball_pos, robot_pos)

                weight = 1/2 + 1/2 * min((dist/radius), 1)

                return weight * 1 if m.ball.y >= 0.65 else 0
            
            return s

        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0, 0.650),
                theta = math.pi/2,
                line_size = 0.25,
                line_size_max = 0.25,
                line_dist = 0.25,
                line_dist_max = 0.25,
                line_dist_single_side = True,
                decay = lambda x: (-x**2) + 1,
                multiplier = self.obey_rules_speed
            )
        )
        
        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (-0.2, 0.650),
                theta = 0,
                line_size = 0.2,
                line_size_max = 0.2,
                line_dist = 0.2,
                line_dist_max = 0.2,
                decay = lambda x: x**2,
                multiplier = self.obey_rules_speed * 1.5
            )
        )

        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (2*0.750, 0.650),
                theta = 3*math.pi/2,
                line_size = 0.25,
                line_size_max = 0.25,
                line_dist = 0.25,
                line_dist_max = 0.25,
                line_dist_single_side = True,
                decay = lambda x: (-x**2) + 1,
                multiplier = self.obey_rules_speed
            )
        )
        
        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (2*0.750+0.2, 0.650),
                theta = 2*math.pi,
                line_size = 0.2,
                line_size_max = 0.2,
                line_dist = 0.2,
                line_dist_max = 0.2,
                decay = lambda x: x**2,
                multiplier = self.obey_rules_speed * 1.5
            )
        )

        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0.750, 0.650*2 - 0.1),
                theta = -2*math.pi,
                line_size = 0.750,
                line_size_max = 0.750,
                line_dist = 0.1,
                line_dist_max = 0.1,
                line_dist_single_side = True,
                inverse = True,
                decay = lambda x: x**(0.5),
                multiplier = self.obey_rules_speed * 1.75
            )
        )

        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0.750, 0.1),
                theta = 2*math.pi,
                line_size = 0.750,
                line_dist = 0.1,
                line_dist_max = 0.1,
                line_dist_single_side = True,
                decay = lambda x: x**(0.5),
                multiplier = self.obey_rules_speed * 1.75
            )
        )

        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0.075, 0.85),
                theta = math.pi/2,
                line_size = 0.45,
                line_dist = 0.075,
                line_dist_max = 0.075,
                line_dist_single_side = True,
                line_size_single_side = True,
                decay = lambda x: x**(0.5),
                multiplier = self.obey_rules_speed * 1.8
            )
        )

        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0.075, 0.0),
                theta = math.pi/2,
                line_size = 0.45,
                line_dist = 0.075,
                line_dist_max = 0.075,
                line_dist_single_side = True,
                line_size_single_side = True,
                decay = lambda x: x**(0.5),
                multiplier = self.obey_rules_speed * 1.8
            )
        )

        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (2*(0.75)-0.075, 0.85 + 0.45),
                theta = 3*math.pi/2,
                line_size = 0.45,
                line_dist = 0.075,
                line_dist_max = 0.075,
                line_dist_single_side = True,
                line_size_single_side = True,
                decay = lambda x: x**(0.5),
                multiplier = self.obey_rules_speed * 1.8
            )
        )

        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (2*(0.75)-0.075, 0.0 + 0.45),
                theta = 3*math.pi/2,
                line_size = 0.45,
                line_dist = 0.075,
                line_dist_max = 0.075,
                line_dist_single_side = True,
                line_size_single_side = True,
                decay = lambda x: x**(0.5),
                multiplier = self.obey_rules_speed * 1.8
            )
        )

        self.detain.add_field(self.base_rules)
        self.wait.add_field(self.base_rules)

        self.detain.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target=lambda m: (m.ball.x, m.ball.y + 0.2),                                                                                                                                                                                                                                                                                                                                          
                radius = 0.1,
                radius_max = 2,
                clockwise = True,
                decay=lambda x: 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = pl(self)
            )
        )

        self.detain.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target=lambda m: (m.ball.x, m.ball.y - 0.2),                                                                                                                                                                                                                                                                                                                                          
                radius = 0.1,
                radius_max = 2,
                clockwise = False,
                decay=lambda x: 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = pr(self)
            )
        )

        self.wait.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (0.5, m.ball.y),
                radius = 0.1,
                decay = lambda x: math.log(x)/2 + 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 1
            )
        )

        if self.robot.robot_id != 0:
            self.wait.add_field(
                algorithms.fields.PointField(
                    self.match,
                    target= lambda m: (m.robots[0].x, m.robots[0].y),
                    radius=0.27,
                    radius_max=0.27,
                    decay = lambda x: x**5 - 1,
                    field_limits = [0.75* 2 , 0.65*2],
                    multiplier = 1.5
                )
            )

        if self.robot.robot_id != 1:
            self.wait.add_field(
                algorithms.fields.PointField(
                    self.match,
                    target= lambda m: (m.robots[1].x, m.robots[1].y),
                    radius=0.27,
                    radius_max=0.27,
                    decay = lambda x: x**5 - 1,
                    field_limits = [0.75* 2 , 0.65*2],
                    multiplier = 1.5
                )
            )

        if self.robot.robot_id != 2:
            self.wait.add_field(
                algorithms.fields.PointField(
                    self.match,
                    target= lambda m: (m.robots[2].x, m.robots[2].y),
                    radius=0.27,
                    radius_max=0.27,
                    decay = lambda x: x**5 - 1,
                    field_limits = [0.75* 2 , 0.65*2],
                    multiplier = 1.5
                )
            )

        self.defend.add_field(self.base_rules)

        self.defend.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0.05, 0.650),
                theta = math.pi/2,
                line_size = 0.25,
                line_size_max = 0.25,
                line_dist = 0.25,
                line_dist_max = 0.25,
                line_dist_single_side = True,
                decay = lambda x: (-x**2) + 1,
                multiplier = self.obey_rules_speed * 1.2
            )
        )

        self.defend.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target=lambda m: (
                    m.ball.x + (math.cos(math.pi/3) if m.ball.y < 0.65 else math.cos(5*math.pi/3)) * 0.1,
                    m.ball.y + (math.sin(math.pi/3) if m.ball.y < 0.65 else math.sin(5*math.pi/3)) * 0.1
                ),                                                                                                                                                                                                                                                                                                                                          
                radius = 0.04,
                radius_max = 2,
                clockwise = lambda m: (m.ball.y < 0.65),
                decay=lambda x: 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 1
            )
        )


    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot


    def decide(self):
        dist_to_ball = np.linalg.norm(
            np.array([self.robot.x, self.robot.y]) - 
            np.array([self.match.ball.x, self.match.ball.y])
        )
        ball = [self.match.ball.x, self.match.ball.y]
        goal_area = [-0.05, 0.30, 0.20, 0.70]
        dist_to_ball_goal = math.sqrt(
            (0 - self.match.ball.x)**2 + (0.65 - self.match.ball.y)**2
        )

        behaviour = None

        if point_in_rect(ball ,goal_area):
            behaviour = self.wait
        elif dist_to_ball_goal <= 0.65 and self.match.ball.x <= 0.35:
            behaviour = self.defend
        elif self.match.ball.x <= 0.750:
            behaviour = self.detain
        else:
            behaviour = self.wait
        
        if self.exporter:
            self.exporter.export(behaviour, self.robot, self.match.ball)

        return behaviour.compute([self.robot.x, self.robot.y])

