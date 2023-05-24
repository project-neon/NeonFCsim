import math
import numpy as np

def angle_adjustment(angle):
        """Adjust angle of the robot when objective is "behind" the robot"""
        phi = angle % math.radians(360)
        if phi > math.radians(180):
            phi = phi - math.radians(360)

        return phi

class PID_control_2(object):
    def __init__(self, robot, default_fps=60,tf=True, max_speed=1.8,smooth_w=0, max_angular=2400,KB=0, krho=100, kp=60, ki=0, kd=0, reduce_speed=False):
        self.vision = robot.game.vision
        self.field_w, self.field_h = robot.game.field.get_dimensions()
        self.robot = robot
        self.desired = [0, 0]
        self.desire_angle = 0
        self.max_angular = max_angular
        self.two_face = tf
        self.smooth_w = smooth_w
        self.w = 0
        self.V = 0
        self.ball = self.robot.strategy.match.ball
        self.right = True
        self.reduce_speed = reduce_speed

        self.l = self.robot.dimensions.get('L')/2 # half_distance_between_robot_wheels
        self.R = self.robot.dimensions.get('R')   # radius of the wheel

        self.default_fps = default_fps
        self.dt = 1/self.default_fps

        # Control params
        self.K_RHO = krho # Linear speed gain

        # PID of angular speed
        self.KP = kp # Proportional gain of w (angular speed), respecting the stability condition: K_RHO > 0 and KP > K_RHO
        self.KI = ki # Integral gain of w 
        self.KD = kd # Derivative gain of w
        self.KB = KB

        # PID params for error
        self.dif_alpha = 0 # diferential param
        self.int_alpha = 0 # integral param
        self.alpha_old = 0 # stores previous iteration alpha
        self.alpha = 0
        # Max speeds for the robot
        self.v_max = max_speed # linear speed 
        self.w_max = math.radians(max_angular) # angular speed rad/s
        self.w_speed = 0
    def set_desired(self, vector):
        self.desired = vector
    def set_angle(self, a):
        self.desire_angle = a
    def _update_fps(self):
        if self.vision._fps > 0: 
            self.dt = 1/self.vision._fps
        else:
            self.dt = 1/self.default_fps

    def update(self):
        # Params calculation
        # Feedback errors
        
        D_x =  self.desired[0] - self.robot.x
        D_y =  self.desired[1] - self.robot.y

        # RHO distance of the robot to the objective
        rho = math.sqrt((D_x**2 + D_y**2))

        # GAMMA robot's position angle to the objetive
        gamma = angle_adjustment(math.atan2(D_y, D_x))
        
        # ALPHA angle between the front of the robot and the objective
        self.alpha = angle_adjustment(gamma - self.robot.theta)

        # BETA
        beta = angle_adjustment(self.desire_angle - gamma)

        """Calculate the parameters of PID control"""
        self._update_fps()
        self.dif_alpha = self.alpha - self.alpha_old / self.dt # Difentential of alpha
        self.int_alpha = self.int_alpha + self.alpha

        """Linear speed (v)"""
        if self.reduce_speed:
            # v = self.v_max if [self.robot.x, self.robot.y] < [self.ball.x, self.ball.y] else min(self.K_RHO*rho, self.v_max)
            v = min(self.K_RHO*rho, self.v_max)
        else:
            v = self.v_max
        
        # """Objective behind the robot"""
        dt = 1
        if self.two_face and(abs(self.alpha) > 2*math.pi/3):
            self.right = True
        elif self.two_face and(abs(self.alpha) < math.pi/3):
            self.right = False


        if self.right:
            self.V -= np.sign(v)*dt
            beta = angle_adjustment(self.desire_angle - math.pi - gamma)
            self.alpha = angle_adjustment(self.alpha - math.pi)
        if not self.right:
            self.V += np.sign(v)*dt
            beta = angle_adjustment(self.desire_angle - gamma)

        


        self.V = np.sign(self.V)*min(self.v_max, abs(self.V))

        self.w_max = math.radians(self.max_angular - self.smooth_w*(self.robot.vx**2 + self.robot.vy**2)**(1/2))

        """Angular speed (w)"""
        self.w = self.KP * self.alpha + self.KB * beta + self.KI * self.int_alpha + self.KD * self.dif_alpha
        self.w = np.sign(self.w) * min(abs(self.w), self.w_max)
        
        self.alpha_old = self.alpha

        """Wheel power calculation"""
        pwr_left = (2 * self.V - self.w * self.l)/2 * self.R
        pwr_right = (2 * self.V + self.w * self.l)/2 * self.R

        return pwr_left * 1000, pwr_right * 1000



