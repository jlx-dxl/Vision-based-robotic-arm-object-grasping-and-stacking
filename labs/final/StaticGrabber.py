import sys
import numpy as np
from copy import deepcopy
from math import pi

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

from lib.IK_position_null import IK
from lib.calculateFK import FK


class StaticGrabber():
    def __init__(self, detector, arm, team, ik, fk):
        self.detector = detector
        self.arm = arm
        self.team = team
        self.ik = ik
        self.fk = fk
        if team == 'blue':
        # 1 7 joint, minus is close to blue des, plus is to static blocks
        # 2 joint, plus is bend down, minus is lay up
        # 4 6, plus is look up, minus is look down
            # self.over_blk = np.array([-0.01779206+0.4, -0.76012354+0.65,  0.01978261, -2.34205014+0.4, 0.02984053, 1.54119353+0.35, 0.75344866+0.4])
            self.over_blk = np.array([
                                [ 0.0337, -0.159 ,  0.1539, -2.0619,  0.0257,  1.9047,  0.9626] ,
                                [ 0.2275, -0.0073,  0.2354, -1.8905,  0.0018,  1.8834,  1.2477] ,
                                [ 0.1383,  0.1481,  0.029 , -1.6939, -0.0044,  1.842 ,  0.9535] ,
                                [ 0.3408,  0.2899,  0.0638, -1.4989, -0.0187,  1.7882,  1.1914] 
                                ])

            self.set_point = np.array([
                                [-0.0975,  0.2073, -0.1692, -2.0558,  0.0449,  2.2597,  0.4937] ,
                                [-0.1087,  0.1437, -0.1578, -2.0025,  0.0268,  2.1443,  0.506 ] ,
                                [-0.1168,  0.0973, -0.1489, -1.9295,  0.016 ,  2.0256,  0.5133] ,
                                [-0.1218,  0.0688, -0.1432, -1.8359,  0.0104,  1.904 ,  0.5173] ,
                                [-0.1241,  0.0593, -0.1411, -1.7201,  0.0085,  1.7789,  0.5186] ,
                                [-0.1248,  0.0708, -0.1425, -1.578 ,  0.0101,  1.6482,  0.5177] ,
                                [-0.1261,  0.1076, -0.1469, -1.401 ,  0.0158,  1.5075,  0.5143] ,
                                [-0.1344,  0.1814, -0.1515, -1.1668,  0.0279,  1.3463,  0.5082] ,
                                ])
        else:
            # 1 7 joint, plus is close to red des, minus is to static blocks
            # 2 joint, plus is bend down, minus is lay up
            # 4 6, plus is look up, minus is look down
            # self.over_blk = np.array([-0.01779206-0.4, -0.76012354+0.65,  0.01978261, -2.34205014+0.4, 0.02984053, 1.54119353+0.35, 0.75344866-0.4])
            # self.over_blk = np.array([-0.01779206-0.4, -0.76012354+0.65+0.1,  0.01978261, -2.34205014+0.4+0.2, 0.02984053, 1.54119353+0.35-0.15, 0.75344866-0.4])
            self.over_blk = np.array([
                                [-0.1333, -0.1573, -0.0603, -2.0619, -0.01  ,  1.9049,  0.5958] ,
                                [-0.232 , -0.0073, -0.2309, -1.8905, -0.0018,  1.8834,  0.323 ] ,
                                [-0.0606,  0.1489, -0.1127, -1.6939,  0.0173,  1.8418,  0.6087] ,
                                [-0.2217,  0.2946, -0.2052, -1.4986,  0.0606,  1.787 ,  0.3541] 
                                ])

            self.set_point = np.array([
                                [ 0.2318,  0.2045,  0.0301, -2.056 , -0.0079,  2.2604,  1.0517] ,
                                [ 0.1986,  0.1423,  0.0645, -2.0026, -0.0109,  2.1445,  1.0538] ,
                                [ 0.1755,  0.0966,  0.0882, -1.9295, -0.0095,  2.0257,  1.0528] ,
                                [ 0.1619,  0.0684,  0.102 , -1.8359, -0.0074,  1.904 ,  1.0514] ,
                                [ 0.1574,  0.0591,  0.1067, -1.7201, -0.0064,  1.7789,  1.0507] ,
                                [ 0.1628,  0.0705,  0.1026, -1.5781, -0.0072,  1.6482,  1.0512] ,
                                [ 0.1799,  0.107 ,  0.0881, -1.4011, -0.0094,  1.5076,  1.0524] ,
                                [ 0.2123,  0.1799,  0.058 , -1.1669, -0.0106,  1.3465,  1.0525] ,
                                ])
        self.H_ee_camera = detector.get_H_ee_camera()
        
        
    def move_to_over(self,i):
        self.arm.safe_move_to_position(self.over_blk[i,:])
        
        
    def detect_and_convert(self):
        result_list = []
        q = self.arm.get_positions()
        # print("current q:", q)
        _, current_H = self.fk.forward(q)
        # print("current_H:", current_H, len(current_H))
        for (_, pose) in self.detector.get_detections():
            # print(pose)
            # print("np.matmul(self.H_ee_camera, pose)", np.matmul(self.H_ee_camera, pose))
            result_list.append(np.matmul(current_H, np.matmul(self.H_ee_camera, pose)))
            # print(np.matmul(current_H, np.matmul(self.H_ee_camera, pose)))
        print(len(result_list)," blocks are detected!!")
        return result_list


    def find_closest(self,transform_matrices):
        # 初始化最小距离和索引
        min_distance = np.inf
        closest_index = -1

        # 遍历所有变换矩阵
        for index, matrix in enumerate(transform_matrices):
            # 提取位移向量（矩阵的第四列前三个元素）
            displacement = matrix[:3, 3]
            # 计算距离
            distance = np.linalg.norm(displacement)

            # 更新最小距离和索引
            if distance < min_distance:
                min_distance = distance
                closest_index = index

        return transform_matrices[closest_index]
    
    
    def solve_pose(self, T_w_closest):
        # 我们需要从这个T_w_closest提取出来两件事，一个是方块位置，一个是和现在的ee的角度差
        # 位置非常直观。就是T_w_closest的前三行的最后一列。
        t_w_closest = T_w_closest[:3, 3]  # 最近的方块在世界坐标系的位置
        # above_blk_position = t_w_closest + np.array([0, 0, 0.1])  # 在方块上方0.2m的位置
        axis = []
        # orientation稍微麻烦
        # 这个旋转矩阵的某一列一定为0 0 1或者0 0 -1。因为正方体的有一个面一定是平行于地面。
        # 那么我只要找到其他任意一个列，就是立方体侧面的方向。这个列会是a b 0的形式。
        # 所以我现在需要旋转我的ee，使得它的X轴方向和这一列重合。
        # EE的X轴方向一旦确定，Z轴方向也是确定的0 0 -1，那么y轴方向就确定。
        for i in range(3):
            if T_w_closest[2][i] < 1e-3 and T_w_closest[2][i] > -1e-3:
                xx = T_w_closest[:3, i]
                if xx[0] < 0:
                    xx = -xx
                axis.append(xx)
                
        print(axis)
            
        candidate_1 = axis[0]
        candidate_2 = axis[1]  
        
        if  candidate_1[0] > candidate_2[0]:
            x = candidate_1
        else:
            x = candidate_2
        
        z = np.array([0, 0, -1])
        y = -np.cross(x, z)
        # 这就是ee的三根轴在世界系的方向。
        above_blk_orientation = np.array([x, y, z]).T  # 组合起来就是ee在世界系的方向

        # above_blk_orientation和above_blk_position组合成target_pose，ik求出对应的q，safe_move_to_position
        # 保持方向不动。垂直下降。再ik一次求出抓取时的q。
        T_at_blk = np.zeros((4,4))
        T_at_blk[:3, :3] = above_blk_orientation
        T_at_blk[:3, 3] = t_w_closest
        return T_at_blk
    
    ##########################################1206 Wenhao Wang Optimize##############################################
    
    def find_axis(self, matrix):
        # Target vectors
        target_pos = np.array([0, 0, 1])
        target_neg = np.array([0, 0, -1])

        # Initialize minimum distance and the corresponding vector index
        min_distance = float('inf')
        closest_vector_index = -1

        # Iterate through each column of the matrix
        for i in range(3):
            vector = matrix[:, i]
            distance_pos = np.linalg.norm(vector - target_pos)
            distance_neg = np.linalg.norm(vector - target_neg)

            # Find the smaller of the two distances (closer to either target_pos or target_neg)
            distance = min(distance_pos, distance_neg)

            # Update minimum distance and index if current vector is closer
            if distance < min_distance:
                min_distance = distance
                closest_vector_index = i

        # Extract the other two vectors
        other_vectors = [matrix[:, i]
                        for i in range(3) if i != closest_vector_index]

        # Choose the vector that is closest to the x axis
        v1 = other_vectors[0]
        v2 = other_vectors[1]
        chosen_axis = None
        if abs(v1[0]) > abs(v2[0]):
            chosen_axis = v1
        else:
            chosen_axis = v2

        # Choose the vector that is closest to the +x vector
        if chosen_axis[0] < 0:
            chosen_axis = -chosen_axis

        x = chosen_axis
        x[2] = 0
        normed_x = x / np.linalg.norm(x)

        return normed_x


    def find_ee_position(self, T_base_blk):
        t_base_ee = T_base_blk[:3, 3]
        return t_base_ee


    def find_ee_orientation(self, T_base_blk):
        R_base_blk = T_base_blk[:3, :3]
        x = self.find_axis(R_base_blk)
        z = np.array([0, 0, -1])
        y = np.cross(z, x)
        R_base_ee = np.array([x, y, z]).T
        return R_base_ee


    def find_target_ee_pose(self, T_base_blk):
        t_base_ee = self.find_ee_position(T_base_blk)
        R_base_ee = self.find_ee_orientation(T_base_blk)
        T_base_ee = np.eye(4)
        T_base_ee[:3, :3] = R_base_ee
        T_base_ee[:3, 3] = t_base_ee
        return T_base_ee
    
    ##########################################1206 Wenhao Wang Optimize##############################################
    
    
    def solve_ik(self, target, seed):
        print("solving IK ...")
        q, _, success_pseudo, message_pseudo = self.ik.inverse(target, seed, method='J_pseudo', alpha=0.75, lamda=0)
        print(success_pseudo, message_pseudo)
        return q
        
    def grab(self, target_H, i):
        # target_H = self.find_closest(result_list)
        # # print("target_H:", target_H)

        target_H_at = self.find_target_ee_pose(target_H)
        # print("target_H:", target_H)
        target_q_at = self.solve_ik(target_H_at, self.arm.get_positions())
        # print("target_q:", target_q)

        print("check!!!")
        # target_q_above = deepcopy(target_q_at)
        # target_q_above[3] += 0.2
        # self.arm.safe_move_to_position(target_q_above)
        self.arm.safe_move_to_position(target_q_at)
        self.arm.exec_gripper_cmd(0.04, 80)
        self.arm.safe_move_to_position(self.over_blk[i,:])
        
    def put(self,i):
        self.arm.safe_move_to_position(self.set_point[i,:])
        # self.arm.exec_gripper_cmd(1.0, 80)
        self.arm.open_gripper()
        safe_position = deepcopy(self.set_point[i,:])
        safe_position[3] += 0.2
        safe_position[5] -= 0.2
        self.arm.safe_move_to_position(safe_position)