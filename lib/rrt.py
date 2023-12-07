import numpy as np
# from tqdm import tqdm
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from lib.calculateFK import FK
from copy import deepcopy

class TreeNode:
    def __init__(self, q):
        self.configurations = q
        self.children = []
        self.parent = None
    
    def add_child(self, child):
        child.parent = self
        self.children.append(child)
    
    def root_path(self):
        if self.parent is None:
            return [self.configurations]
        else:
            return self.parent.root_path() + [self.configurations]
    
    def find_closest(self, target):
        distance = np.linalg.norm(self.configurations - target)
        node = self
        for child in self.children:
            child_distance, child_node = child.find_closest(target)
            if child_distance < distance:
                distance = child_distance
                node = child_node
        return distance, node
    
    
class RRT:
    def __init__(self, map, start, goal, n_sample=1000, step_size=1.5):
        self.map = map
        self.start = start
        self.root = TreeNode(start)
        self.node_goal = TreeNode(goal)
        self.goal = goal
        self.n_sample = n_sample
        self.step_size = step_size
        self.path = []
        self.lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
        self.upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])
        if(self.check_start_and_goal()):
            self.plan()
            # self.print_path()
        else:
            print("plan failed due to invalid inputs")
        
    
    def check_start_and_goal(self):
        if self.isRobotCollided(self.start):
            print("Invalid Start!")
            return False
        elif self.isRobotCollided(self.goal):
            print("Invalid Goal!")
            return False
        else:
            print("Valid Start and Goal!")
            return True
        
    def random_sample(self):
        q_rand = np.random.uniform(low=self.lowerLim, high=self.upperLim)
        return q_rand, TreeNode(q_rand)
            
    
    def isRobotCollided(self, q):
        obstacles = self.map.obstacles
        if len(obstacles) > 0:
            fk = FK()
            joint_positions, _ = fk.forward(q)
            for obs_idx in range(len(obstacles)):
                box = obstacles[obs_idx, :].copy()
                for i in range(7):
                    linePt1 = joint_positions[i, :].reshape(1, 3).copy()
                    linePt2 = joint_positions[i+1, :].reshape(1, 3).copy()
                    if True in detectCollision(linePt1, linePt2, box):
                        return True
        return False

    def isPathCollided(self, q1, q2):
        step = 0.2
        dq = q2 - q1
        n_steps = int(np.linalg.norm(dq) / step)
        for i in range(n_steps):
            q = q1 + i * step * dq
            if self.isRobotCollided(q):
                return True
        return False

    def walk_toward(self, nearest, sample):
        direction = np.array(sample) - np.array(nearest)
        distance = np.linalg.norm(direction)
        if distance > self.step_size:
            direction = (direction / distance) * self.step_size
        return nearest + direction

    def plan(self):
        for _ in range(self.n_sample):
            q_rand, node_rand = self.random_sample()
            # if self.isRobotCollided(q_rand):
            #     continue
            _, node_nearest = self.root.find_closest(q_rand)
            new_node = self.walk_toward(node_nearest.configurations, node_rand.configurations)
            if self.isRobotCollided(new_node):
                continue
            if self.isPathCollided(new_node, node_nearest.configurations):
                continue
            node_new = TreeNode(new_node)
            node_nearest.add_child(node_new)
            if self.isPathCollided(new_node, self.goal):
                continue
            node_new.add_child(self.node_goal)
            self.path = self.node_goal.root_path()
            break

    def print_path(self):
        print(self.path)
        
def rrt(map, start, goal):
    rrt = RRT(deepcopy(map), deepcopy(start), deepcopy(goal))
    return rrt.path

if __name__ == '__main__':
    # np.random.seed(0)
    map_struct = loadmap("maps/map1.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    # rrt = RRT(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    print('path:', path)
