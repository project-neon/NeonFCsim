from strategy.BaseStrategy import Strategy
from controller import PID_control, PID_control_2
from entities import plays
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook

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
                'max_speed': 2, 'max_angular': 4000, 'kd': 0,  
                'kp': 80,'KB':-40, 'krho': 9,'reduce_speed': False
            }
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
   # def update(self):
    #    return super().update()
    def start(self):
        pass
    def update(self):
        res = [0,0]
        res[0] = self.match.ball.x
        res[1] = self.match.ball.y
        if ((self.robot.x - self.match.ball.x)**2 + (self.robot.y - self.match.ball.y)**2)**(1/2) < 0.1:
            self.robot.strategy.controller.KB = -200
        else:
            self.robot.strategy.controller.KB = -40
        self.robot.strategy.controller.set_angle(0)
        return res


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