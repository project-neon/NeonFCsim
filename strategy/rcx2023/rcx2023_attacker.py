from strategy.BaseStrategy import Strategy
from controller import PID_control
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
            controller = PID_control
            controller_kwargs = {
                'max_speed': 1, 'max_angular': 8400, 'kd': 0,  
                'kp': 180, 'krho': 9,'reduce_speed': True
            }
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
    def update(self):
        return super().update()
    def start(self):
        pass
    def update(self):
        res = []
        res[0] = 0
        res[1] = 0
        return res


class MainAttacker(Strategy):
    def __init__(self, match, name="MainAttacker"):
        super().__init__(match, name, controller=PID_control)
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