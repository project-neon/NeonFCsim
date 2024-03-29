import math
import time
#import strategy

class Playbook(object):
    def __init__(self, coach, log=False):
        self.coach = coach
        self.plays = {}
        self.actual_play = None
        self.log = log
    
    def add_play(self, play):
        self.plays[play.get_name()] = play

    def set_play(self, play):
        if play.get_name() in self.plays.keys():
            self.actual_play = play.get_name()
            play.start_up()
        else:
            raise KeyError("Play is not defined on actual Playbook")
    def get_actual_play(self):
        return self.plays[self.actual_play]

    def _transition_if_have(self):
        for transition, next_play in self.plays[self.actual_play].transitions:
            if transition.evaluate(self.coach, self.plays[self.actual_play]):
                if self.log:
                    pass
                    #print(f"Transition to {next_play.get_name()}")
                self.set_play(next_play)

    def update(self):
        self._transition_if_have()
        self.plays[self.actual_play].update()

class Trigger(object):
    def __init__(self):
        pass

    def evaluate(self, coach, actual_play):
        return False

class Play(object):
    def __init__(self, coach):
        self.coach = coach
        self.transitions = []
        self.start_running_time = 0

    def start_up(self):
        self.start_running_time = time.time()
    
    def get_running_time(self):
        return time.time() - self.start_running_time

    def get_positions(self, foul, team_color, foul_color, quadrant):
        return None

    def get_name(self):
        return self.__class__

    def tear_down(self):
        pass

    def update(self):
        pass
    
    def add_transition(self, trigger, destination):
        self.transitions.append( (trigger, destination) )

class OnWall(Trigger): #Ativado quando a bola e o robo estão nas paredes horizontais e próximos
    def __init__(self,match,robot):
        super().__init__()
        self.robot = robot
        self.match = match
        self.field_dim = self.match.game.field.get_dimensions()
    def evaluate(self,coach,actual_play):
        self.delta = 0.1
        ball_distance = 0.07
        if ((self.robot.x - self.match.ball.x)**2 + (self.robot.y - self.match.ball.y)**2)**(1/2) < ball_distance and (self.match.ball.vx**2+self.match.ball.vy**2)**(1/2) < 0.1:
            if abs(self.field_dim[0] - self.robot.x) < self.delta :
                if abs(self.field_dim[0] - self.match.ball.x) < self.delta:
                    return True
            elif abs(self.robot.x) < self.delta:
                if abs(self.match.ball.x) < self.delta:
                    return True
            if abs(self.field_dim[1] - self.robot.y) < self.delta :
                if abs(self.field_dim[1] - self.match.ball.y) < self.delta:
                    return True
            elif abs(self.robot.y) < self.delta:
                if abs(self.robot.y) < self.delta:
                    return True
        return False
class OnWall2(Trigger): #Ativado quando apenas a bola está nas paredes horizontais
    def __init__(self,match,robot):
        super().__init__()
        self.robot = robot
        self.match = match
        self.field_dim = self.match.game.field.get_dimensions()
    def evaluate(self,coach,actual_play):
        self.delta = 0.2
        ball_distance = 0.1
        if ((self.robot.x - self.match.ball.x)**2 + (self.robot.y - self.match.ball.y)**2)**(1/2) > ball_distance:
            if abs(self.field_dim[1] - self.match.ball.y) < self.delta:
                return True
            if abs(self.match.ball.y) < self.delta:
                return True
        return False
class StaticInWall(Trigger): #Ativado quando o robo está preso
    def __init__(self,match,robot):
        super().__init__()
        self.robot = robot
        self.match = match
        self.field_dim = self.match.game.field.get_dimensions()
    def evaluate(self,coach,actual_play):
        self.delta = 0.3
        self.ball_distance = 0
        if (math.atan((self.robot.y - self.match.ball.y)/(self.robot.x - self.match.ball.x)) - self.robot.theta) > 0.4:
            if abs((self.robot.vx**2 + self.robot.vy**2)**(1/2) < 0.05 and ((self.robot.x - self.match.ball.x)**2 + (self.robot.y - self.match.ball.y)**2)**(1/2)) > self.ball_distance:
                return True
                if abs(self.field_dim[1] - self.robot.y) < self.delta:
                    return True
                if abs(self.robot.y) < self.delta:
                    return True
        return False
class MissBall(Trigger):
    def __init__(self,match,robot):
        super().__init__()
        self.robot = robot
        self.match = match
    def evaluate(self,coach,actual_play):
        if self.robot.x > self.match.ball.x:
            return True
        return False
class OnBall(Trigger):
    def __init__(self,match,robot,distance):
        super().__init__()
        self.robot = robot
        self.match = match
        self.distance = distance
    def evaluate(self,coach,actual_play):
        d = ((self.robot.x-self.match.ball.x)**2 + (self.robot.y-self.match.ball.y)**2)**(1/2)
        if self.robot.x < self.match.ball.x and d < self.distance:
            return True
        return False
class OnPoint(Trigger):
    def __init__(self,match,robot,point,distance):
        super().__init__()
        self.match = match
        self.robot = robot
        self.point = point
        self.distance = distance
    def evaluate(self,coach,actual_play):
        d = ((self.robot.x - self.point[0](self.match))**2 + (self.robot.y - self.point[1](self.match))**2)
        if d <= self.distance:
            return True
        return False
class AttackPossible(Trigger):
    def __init__(self,match,robot):
        super().__init__()
        self.robot = robot
        self.match = match
        self.field_dim = self.match.game.field.get_dimensions()
    def evaluate(self,coach,actual_play):
        if self.robot.x + 0.4 < self.match.ball.x and self.robot.y > 0.2 and self.robot.y < 1.1:
            return True
        return False
class GoalkeeperOut(Trigger):
    def __init__(self,match,robot,dist, invert=False):
        super().__init__()
        self.robot = robot
        self.match = match
        self.invert = invert
        self.dist = dist
        self.field_dim = self.match.game.field.get_dimensions()
    def evaluate(self,coach,actual_play):
        if self.robot.x > self.dist and not self.invert:
            return True
        elif self.robot.x < self.dist and self.invert:
            return True
        return False
class WaitFor(Trigger):
    def __init__(self, timeout):
        """
        stuck_timeout: time to active transition in seconds
        """
        super().__init__()
        self.timeout = timeout
    
    def evaluate(self, coach, actual_play):
        return (self.timeout - actual_play.get_running_time()) <= 0

class IsAttackerSpin(Trigger):
    def __init__(self):
        super().__init__()
        self.timeout = 0
    
    def evaluate(self, coach, actual_play):
        match = coach.match
        is_spinning = False
        for r in match.robots:
            if r.strategy.name == "MainAttacker":
                if r.strategy.playerbook.get_actual_play().get_name() == f"<{r.get_name()} Spinner Planning>":

                    is_spinning = True
                    break
        return is_spinning


class StuckRobots(Trigger):
    def __init__(self, stuck_strategies=[]):
        """
        stuck_timeout: time to active transition in seconds
        """
        super().__init__()
        self.stuck_strategies = stuck_strategies
    
    def evaluate(self, coach, actual_play):
        match = coach.match

        robots = [r for r in match.robots]
        dict_robots = {
            r.strategy.name: r for r in robots if r.strategy
        }

        if all([ dict_robots[st].is_stuck() for st in self.stuck_strategies if dict_robots.get(st)]):
            return True
        
        return False

class OnFreeBall(Trigger):
    def __init__(self, referee):
        super().__init__()
        self.referee = referee

    def evaluate(self, coach, actual_play):
        foul = self.referee.get_foul()
        return foul == "FREE_BALL"

class OnKickOff(Trigger):
    def __init__(self, referee):
        super().__init__()
        self.referee = referee

    def evaluate(self, coach, actual_play):
        foul = self.referee.get_foul()
        return foul == "KICK_OFF"

class OnFreeKick(Trigger):
    def __init__(self, referee, team_color):
        super().__init__()
        self.referee = referee
        self.team_color = team_color

    def evaluate(self, coach, actual_play):
        foul = self.referee.get_foul()
        return foul == "FREE_KICK" and self.team_color.upper() == self.referee.get_color()

class OnPenaltyKick(Trigger):
    def __init__(self, referee, team_color):
        super().__init__()
        self.referee = referee
        self.team_color = team_color

    def evaluate(self, coach, actual_play):
        foul = self.referee.get_foul()
        # print(foul, self.referee.get_color())
        return foul == "PENALTY_KICK" and self.team_color.upper() == self.referee.get_color()

class OnFreeBall(Trigger):
    def __init__(self, referee, team_color):
        super().__init__()
        self.referee = referee
        self.team_color = team_color

    def evaluate(self, coach, actual_play):
        foul = self.referee.get_foul()
        return foul == "FREE_BALL" and self.referee.get_quadrant() in ('QUADRANT_1', 'QUADRANT_4')

class OnFreeBallDef(Trigger):
    def __init__(self, referee, team_color):
        super().__init__()
        self.referee = referee
        self.team_color = team_color

    def evaluate(self, coach, actual_play):
        foul = self.referee.get_foul()
        return foul == "FREE_BALL" and self.referee.get_quadrant() in ('QUADRANT_2', 'QUADRANT_3')

class OnGoalKick(Trigger):
    def __init__(self, referee, team_color):
        super().__init__()
        self.referee = referee
        self.team_color = team_color

    def evaluate(self, coach, actual_play):
        foul = self.referee.get_foul()
        return foul == "GOAL_KICK" and self.team_color.upper() == self.referee.get_color()
