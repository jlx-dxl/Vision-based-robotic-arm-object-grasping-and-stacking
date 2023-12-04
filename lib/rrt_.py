import numpy as np
from tqdm import tqdm
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from lib.calculateFK import FK
from copy import deepcopy

class RRTTreeNode:
    def __init__(self, configuration):
        self.configuration = configuration
        self.children = []
        self.parent = None
    
    def add_child(self, child):
        child.parent = self
        self.children.append(child)
    
    def root_path(self):
        if self.parent is None:
            return [self.configuration]
        else:
            return self.parent.root_path() + [self.configuration]
    
    def find_closest(self, target):
        distance = np.linalg.norm(self.configuration - target)
        node = self
        for child in self.children:
            child_distance, child_node = child.find_closest(target)
            if child_distance < distance:
                distance = child_distance
                node = child_node
        return distance, node
    

def if_node_collided(map, q):
    obstacles = map.obstacles
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

def if_edge_collided(map, q1, q2):
    step = 0.01
    dq = q2 - q1
    n_steps = int(np.linalg.norm(dq) / step)
    for i in range(n_steps):
        q = q1 + i * step * dq
        if if_node_collided(map, q):
            return True
    return False


def random_sample():
    lowerbound = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upperbound = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])
    q_rand = np.random.uniform(low=lowerbound, high=upperbound)
    return q_rand

def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (0x7).
    :param goal:        goal pose of the robot (0x7).
    :return:            returns an mx7 matrix, where each row consists of the configuration of the Panda at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is empty
    """

    path = []
    n_samples = 1000
    

    if if_node_collided(map, start):
        print("Invalid Start Point!")
        return path
    elif if_node_collided(map, goal):
        print("Invalid Goal Point!")
        return path
    else :
        print("Valid Start and Goal Points!")
        root = RRTTreeNode(start)
        
        # for _ in range(n_samples):
        for _ in tqdm(range(n_samples)):
            q_rand = random_sample()
            if if_node_collided(map, q_rand):
                continue
            
            _, node_nearest = root.find_closest(q_rand)
            if if_edge_collided(map, node_nearest.configuration, q_rand):
                continue
            node_rand = RRTTreeNode(q_rand)
            node_nearest.add_child(node_rand)

            if if_edge_collided(map, q_rand, goal):
                continue

            node_goal = RRTTreeNode(goal)
            node_rand.add_child(node_goal)
            path = node_goal.root_path()
            return np.array(path)
    
    return np.array(path)

if __name__ == '__main__':
    map_struct = loadmap("maps/map1.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    print('path:', path)