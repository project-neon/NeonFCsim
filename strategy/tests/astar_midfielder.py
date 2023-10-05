from strategy.BaseStrategy import Strategy
from controller import PID_control, PID_control_2, TwoSidesLQR, PID_control_3
from entities import plays
from algorithms.potential_fields import fields
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook, OnInsideBox, OnRobotInsideBox, OnField
from entities.plays.playbook import MissBall, OnBall, AttackPossible, OnPoint, StaticInWall, WaitFor
import numpy as np
from algorithms.astar import AStar, Node
from algorithms.astar.fieldGraph import FieldGraph
from scipy.spatial import Voronoi
from commons import math

def astar_path(match, robot, target, target_is_ball=False):
    robot_node = Node([robot.x, robot.y])
    robot_node.position = [robot.x, robot.y]

    graph = FieldGraph()
    graph.set_start(robot_node)

    field_param = match.game.field.get_dimensions()

    corners = [
        [field_param[0]/2, 0],
        [0, 0],
        [0, field_param[1]],
        [field_param[0]/2, field_param[1]],

        [0, field_param[1]/2],
        [0, field_param[1]/4],
        [0, field_param[1]*3/4]
    ]

    mergeble_obstacles = [ [r.x, r.y] for r in match.opposites] + [
        [r.x, r.y] for r in match.robots 
        if r.robot_id != robot.robot_id
    ] + [[match.ball.x, match.ball.y]]

    dist = lambda x, y: np.sqrt( ( (x[0] - y[0])**2 + (x[1] - y[1])**2 ) )
    to_merge = []
    to_remove = []
    for i1, m1 in enumerate(mergeble_obstacles):
        for i2, m2 in enumerate(mergeble_obstacles):
            if m1 == m2: continue
            if dist(m1, m2) <= 2 * robot.dimensions["L"] * np.sqrt(2):
                to_merge.append([m1, m2])
                to_remove.append(i1)
                to_remove.append(i2)
    
    to_remove = list(set(to_remove))
    for index in sorted(to_remove, reverse=True):
        del mergeble_obstacles[index]

    for o1, o2 in to_merge:
        mergeble_obstacles.append([ (o1[0] + o2[0])/2 , (o1[1] + o2[1])/2 ])

    objective = target # [x, y]

    unmergeble_obsctacles = [target, robot_node.position]

    obstacles = corners + mergeble_obstacles + unmergeble_obsctacles

    vor = Voronoi(obstacles)

    target_node = Node(objective)

    nodes = [
        Node([a[0], a[1]]) for a in vor.vertices
    ] + [
        target_node, robot_node
    ]

    objective_index = len(obstacles) - 2
    robot_index = len(obstacles) - 1
    
    graph.set_nodes(nodes)

    polygon_objective_edges = []
    polygon_robot_edges = []

    for edge, ridge_vertice in zip(vor.ridge_vertices, vor.ridge_points):
        if edge[0] == -1: continue
        graph.add_edge([nodes[edge[0]], nodes[edge[1]]])

        if objective_index in ridge_vertice:
            polygon_objective_edges.append(nodes[edge[0]])
            polygon_objective_edges.append(nodes[edge[1]])

        if robot_index in ridge_vertice:
            polygon_robot_edges.append(nodes[edge[0]])
            polygon_robot_edges.append(nodes[edge[1]])
        
        if objective_index in ridge_vertice and robot_index in ridge_vertice:
            graph.add_edge([robot_node, target_node])

    for edge_to_target in set(polygon_objective_edges):
        graph.add_edge([edge_to_target, target_node])

    for edge_to_target in set(polygon_robot_edges):
        graph.add_edge([edge_to_target, robot_node])

    astar = AStar(robot_node, target_node)

    astar_path = astar.calculate()

    if astar_path:
        return astar_path[1]
    else:
        print("NO PATH TO TARGET")
        return [robot.x, robot.y]

class AstarToBall(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
        self.fsize = self.match.game.field.get_dimensions()

    def get_name(self):
        return f"<{self.robot.get_name()} AstarToBall>"
    
    def start_up(self):
            super().start_up()
            controller = PID_control
            controller_kwargs = {
                'max_speed': 2.5, 'max_angular': 4000, 'kd': 0,  
                'kp': 120, 'krho': 8,'reduce_speed': False
            }

            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
            
    # def update(self):
    #     return super().update()
    
    def start(self, robot=None):
        pass
    
    def update(self, goal = []):
        if goal:
            target = goal
        else:
            target = [self.match.ball.x - 0.06, self.match.ball.y]
        res = astar_path(self.match, self.robot, target, True)
        return res
    
class waitplay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
        self.fsize = self.match.game.field.get_dimensions()

    def get_name(self):
        return f"<{self.robot.get_name()} waitplay>"
    
    def start_up(self):
            super().start_up()
            controller = PID_control
            controller_kwargs = {
                'max_speed': 5, 'max_angular': 4000, 'kd': 0,
                'kp': 80, 'krho': 8,'reduce_speed': True
            }

            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    # def update(self):
    #     return super().update()

    def start(self):
        pass

    def update(self):
        res = [(self.fsize[0]/2)-0.18, self.fsize[1]/2]
        return res

class pushball(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
        self.fsize = self.match.game.field.get_dimensions()

    def get_name(self):
        return f"<{self.robot.get_name()} pushball>"
    
    def start_up(self):
            super().start_up()
            controller = PID_control_3
            controller_kwargs = {
                'max_speed': 3, 'max_angular': 4000, 'kd': 0,
                'kp': 80, 'krho': 8,'reduce_speed': False,
                'two_sides': True
            }

            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    # def update(self):
    #     return super().update()

    def start(self):
        pass

    def update(self):
        self.robot.strategy.spin = 2000
        res = [self.match.ball.x, self.match.ball.y]
        return res

class AvoidAreaSide(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
        self.fsize = self.match.game.field.get_dimensions()

    def get_name(self):
        return f"<{self.robot.get_name()} AvoidAreaSide>"
    
    def start_up(self):
            super().start_up()
            controller = PID_control_3
            controller_kwargs = {
                'max_speed': 5, 'max_angular': 4000, 'kd': 0,  
                'kp': 80, 'krho': 8,'reduce_speed': False,
                'two_sides': True
            }

            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
            
    # def update(self):
    #     return super().update()
    
    def start(self):
        pass
    
    def update(self):
        res = [(self.fsize[0]/2)-0.18, self.fsize[0] / 2]
        return res

class AvoidAreaTop(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
        self.fsize = self.match.game.field.get_dimensions()

    def get_name(self):
        return f"<{self.robot.get_name()} AvoidAreaTop>"
    
    def start_up(self):
            super().start_up()
            controller = PID_control_3
            controller_kwargs = {
                'max_speed': 5, 'max_angular': 4000, 'kd': 0,  
                'kp': 80, 'krho': 8,'reduce_speed': False,
                'two_sides': True
            }

            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
            
    # def update(self):
    #     return super().update()
    
    def start(self):
        pass
    
    def update(self):
        # res = [self.fsize[0] / 10, self.fsize[0] * 5/6]
        res = [self.fsize[0] - 0.2, 0.35]
        return res

class AvoidAreaBottom(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.robot = robot
        self.fsize = self.match.game.field.get_dimensions()

    def get_name(self):
        return f"<{self.robot.get_name()} AvoidAreaBottom>"
    
    def start_up(self):
            super().start_up()
            controller = PID_control_3
            controller_kwargs = {
                'max_speed': 5, 'max_angular': 4000, 'kd': 0,  
                'kp': 80, 'krho': 8,'reduce_speed': False,
                'two_sides': True
            }

            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
            
    # def update(self):
    #     return super().update()
    
    def start(self):
        pass
    
    def update(self):
        # res = [self.fsize[0] / 10, self.fsize[0] / 6]
        res = [0.2, 0.35]
        return res

class AstarMidfielder(Strategy):
    def __init__(self, match, name="AstarMidfielder"):
        super().__init__(match, name, controller=PID_control)
        self.playerbook = None

    def start(self, robot=None):
        super().start(robot=robot)
        self.fsize = self.match.game.field.get_dimensions()
        self.spin = 0
        self.push = 0
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        inside_our_field = OnField(self.match, 5, self.fsize)
        outside_our_field = OnInsideBox(self.match, [self.fsize[0]/2, 0, self.fsize[0]/2, self.fsize[1]])
        goal_area_side = [-0.15, 0.7, 0.55, 0.4] # x1, y1, w, h
        inside_goal_area_side = OnRobotInsideBox(self.match, goal_area_side, self.robot)
        goal_area_top = [0, 1.1, 0.35, 0.25]
        inside_goal_area_top = OnRobotInsideBox(self.match, goal_area_top, self.robot)
        goal_area_bottom = [0, 0.45, 0.35, 0.25]
        inside_goal_area_bottom = OnRobotInsideBox(self.match, goal_area_bottom, self.robot)
        wait_aa = WaitFor(2)
        near_ball = OnBall(self.match, self.robot, 0.075)
        wait_push = WaitFor(0.2) # 0.2 seconds

        pm = AstarToBall(self.match, self.robot)
        pm.start()
        
        wp = waitplay(self.match, self.robot)
        wp.start()

        push_ball = pushball(self.match, self.robot)
        push_ball.start()

        aa_side = AvoidAreaSide(self.match, self.robot)
        aa_side.start()

        aa_top = AvoidAreaTop(self.match, self.robot)
        aa_top.start()

        aa_bottom = AvoidAreaBottom(self.match, self.robot)
        aa_bottom.start()

        self.playerbook.add_play(pm)
        self.playerbook.add_play(wp)
        self.playerbook.add_play(push_ball)
        self.playerbook.add_play(aa_top)
        self.playerbook.add_play(aa_bottom)
        self.playerbook.add_play(aa_side)

        pm.add_transition(outside_our_field, wp)
        wp.add_transition(inside_our_field, pm)
        pm.add_transition(near_ball, push_ball)
        push_ball.add_transition(wait_push, pm)
        pm.add_transition(inside_goal_area_top, aa_top)
        aa_top.add_transition(wait_aa, pm)
        pm.add_transition(inside_goal_area_bottom, aa_bottom)
        aa_bottom.add_transition(wait_aa, pm)
        pm.add_transition(inside_goal_area_side, aa_side)
        aa_side.add_transition(wait_aa, pm)

        # push.add_transition(self.attack,pred)
        self.playerbook.set_play(pm)

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