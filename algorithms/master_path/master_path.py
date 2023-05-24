
import numpy as np
import matplotlib.pyplot as plt

class MasterPath:
    class box_obstacle:
        def __init__(self,pos=[0,0],size=[0.01,0.01]):
            self.pos = pos
            self.size = size
        def check_collision(self,point):
            if abs(self.pos[0] - point[0]) < self.size[0] and abs(self.pos[1] - point[1]) < self.size[1]:
                return True
            else:
                return False
    class circle_obstacle:
        def __init__(self,pos=[0,0],r=0.01):
            self.pos = pos
            self.r = r
        def check_collision(self,point):
            if ((self.pos[0] - point[0])**2 + (self.pos[1] - point[1])**2)**(1/2) < self.r:
                return True
            else:
                return False
    class semicircle_obstacle:
        def __init__(self,pos=[0,0],r=0.01,orientation="right"):
            self.pos = pos
            self.orientation = orientation
            self.r = r
        def check_collision(self,point):
            if self.orientation == "right":
                if ((self.pos[0] - point[0])**2 + (self.pos[1] - point[1])**2)**(1/2) < self.r and point[0] < self.pos[0]:
                    return True
                else:
                    return False
            else:
                if ((self.pos[0] - point[0])**2 + (self.pos[1] - point[1])**2)**(1/2) < self.r and point[0] > self.pos[0]:
                    return True
                else:
                    return False
    class node:
        def __init__(self,pos):
            self.pos = pos
            #self.last_node
    def __init__(self, goal=[1,1], point=[0,0],stepsize=0.005,delta=0.01):
        self.goal = goal
        self.point = point
        self.current = 0
        self.path = []
        self.obstacles = []
        self.stepsize = stepsize
        self.delta = delta

    def add_obstacle(self,obstacle):
        self.obstacles.append(obstacle)

    def check_collision(self,point):
        for i in self.obstacles:
            if i.check_collision(point):
                return True
        return False

    def generate_path(self):
        self.current = 0
        self.pathing = []
        for i in range(300):
            if self.pathing == []:
                self.pathing = [self.node(self.point)]
            elif ((self.pathing[len(self.pathing)-1].pos[0] - self.goal[0])**2 + (self.pathing[len(self.pathing)-1].pos[1] - self.goal[1])**2)**(1/2) > self.delta:
                self.pathing.append(self.node(self.generate_point(self.pathing)))
        return self.pathing

    def generate_point(self,p):
        nextx = (self.goal[0] - p[len(p)-1].pos[0])
        nexty = (self.goal[1] - p[len(p)-1].pos[1])
        nextx /= ((self.goal[0] - p[len(p)-1].pos[0])**2 + (self.goal[1] - p[len(p)-1].pos[1])**2)**(1/2)
        nexty /= ((self.goal[0] - p[len(p)-1].pos[0])**2 + (self.goal[1] - p[len(p)-1].pos[1])**2)**(1/2)
        nextx *= self.stepsize
        nexty *= self.stepsize
        theta = 0
        for i in range(314):
            po = [p[len(p)-1].pos[0] + np.cos(theta)*nextx - np.sin(theta)*nexty, p[len(p)-1].pos[1] + np.sin(theta)*nextx + np.cos(theta)*nexty]
            if not self.check_collision(po):
                break
            else:
                if theta >= 0 and abs(theta) < 3.14/2:
                    theta = -(theta + 0.04)
                elif abs(theta) < 3.14/2:
                    theta = -(theta - 0.04)
                else:
                    theta = 0
                    break
                #print(theta)
            po = [p[len(p)-1].pos[0] + np.cos(theta)*nextx - np.sin(theta)*nexty, p[len(p)-1].pos[1] + np.sin(theta)*nextx + np.cos(theta)*nexty]
        return po
    def next(self):
        if self.current < len(self.path) - 1:
            self.current += 1
    def plot(self):
        x=[]
        y=[]
        for i in self.path:
            x.append(i.pos[0])
            y.append(i.pos[1])
        return [x,y]
    def pathsmoothing(self,k,n,inversus,commum):
        if inversus:
          inv = self.path[::-1]
          n = int(np.floor(n/self.stepsize))
          for i in range(len(inv)) :
              if i < len(self.path):
                s = []
                for num in range(n):
                  if i-2-num > 0:
                    ux = inv[i-1-num].pos[0] - inv[i-2-num].pos[0]
                    uy = inv[i-1-num].pos[1] - inv[i-2-num].pos[1]
                    vx = inv[i].pos[0] - inv[i-1].pos[0]
                    vy = inv[i].pos[1] - inv[i-1].pos[1]
                    dx = -vx + ux 
                    dy = -vy + uy
                    s.append([dx,dy])
                for d in s:
                  inv[i].pos[0] += d[0]*k/len(s)
                  inv[i].pos[1] += d[1]*k/len(s)
          self.path = inv[::-1]
        if commum:
          c = self.path
          n = int(np.floor(n/self.stepsize))
          for i in range(len(c)) :
              if i < len(self.path):
                s = []
                for num in range(n):
                  if i-2-num > 0:
                    ux = c[i-1-num].pos[0] - c[i-2-num].pos[0]
                    uy = c[i-1-num].pos[1] - c[i-2-num].pos[1]
                    vx = c[i].pos[0] - c[i-1].pos[0]
                    vy = c[i].pos[1] - c[i-1].pos[1]
                    dx = -vx + ux 
                    dy = -vy + uy
                    s.append([dx,dy])
                for d in s:
                  c[i].pos[0] += d[0]*k/len(s)
                  c[i].pos[1] += d[1]*k/len(s)
          self.path = c
    def merge(self, one, two, k, n):
          inv = one[::-1]
          inv2 = two[::-1]
          n = int(np.floor(n/self.stepsize))
          if len(inv2) > len(inv):
            inv += inv2[len(inv):len(inv2)]
          elif len(inv) > len(inv2):
            for i in range(len(inv) - len(inv2)):
              if inv != []:
               inv.pop()
          for i in range(len(inv2)) :
              if i < len(inv2):
                s = []
                for num in range(n):
                  if i-num > 0 and i-num < len(inv2) and i-num < len(inv) :
                    #print(len(inv),"::",len(inv2))
                    ux = inv2[i-num].pos[0] - inv[i-num].pos[0]
                    uy = inv2[i-num].pos[1] - inv[i-num].pos[1]
                    s.append([ux,uy])
                for d in s:
                  inv2[i].pos[0] -= d[0]*k/n
                  inv2[i].pos[1] -= d[1]*k/n
          return inv2[::-1]
