import numpy as np
from math import pi

class FK():

    def __init__(self):
        """
        initialize DH parameters
        """
        self.q_bias = np.array([0,0,0,0,np.pi,0,-np.pi,-np.pi/4])
        self.d_list = [0.141, 0.192, 0, 0.195+0.121, 0, 0.125+0.259, 0, 0.051+0.159]
        self.p_list = [0, -np.pi/2, np.pi/2, np.pi/2, np.pi/2, -np.pi/2, np.pi/2, 0]
        self.a_list = [0, 0, 0, 0.0825, 0.0825, 0, 0.088, 0]
        self.joint_bias = [np.array([0, 0, 0, 1]),np.array([0, 0, 0, 1]),np.array([0, 0, 0.195, 1]),np.array([0, 0, 0, 1]),
                           np.array([0, 0, 0.125, 1]),np.array([0, 0, -0.015, 1]),np.array([0, 0, 0.051, 1]),np.array([0, 0, 0, 1])]

    def forward(self, q):
        """
        calculate forward kinematics
        INPUT:
        q: (7,) joint angles
        OUTPUT:
        jointPositions: (8,3) actual joint positions
        Tee = Homogenous Trasformation Matrix of the end effector with respect to the world frame
        """
        q_list = np.concatenate(([0],q))
        q_list = q_list + self.q_bias

        A_list = self.compute_Ai(q_list)    
        T_list = self.compute_Hi(A_list)
        jointPositions = self.position_correction(T_list)

        return jointPositions, T_list[-1]


    def trans_matrix(self, q, d, p, a):
        """
        INPUT:
        q : z-angle (input)
        d : z-distance (parameter)
        p : x-angle (parameter)
        a : x-distance (parameter)

        OUTPUTS:
        T : a 4 x 4 homogeneous transformation matrix
        """
        T = [[np.cos(q),-np.sin(q)*np.cos(p),np.sin(q)*np.sin(p),a*np.cos(q)],
             [np.sin(q),np.cos(q)*np.cos(p),-np.cos(q)*np.sin(p),a*np.sin(q)],
             [0,np.sin(p),np.cos(p),d],
             [0,0,0,1]]
        
        return T
    
    def compute_Ai(self,q_list):
        """
        INPUT:
        q_list: (8,) with a 0 at first
        OUTPUT: 
        A_list: list of homogenous transformation matrixes between the neighbor joints
        """
        A_list = []
        for i in range(8):
            A_list.append(self.trans_matrix(q_list[i], self.d_list[i], self.p_list[i], self.a_list[i]))
        return A_list
    
    def compute_Hi(self, A_list):
        """
        INPUT:
        A_list: list of homogenous transformation matrixes between the neighbor joints
        OUTPUT: 
        T_list: list of homogenous transformation matrixes of each joint with respect to the world frame
        """
        T_list = []
        for i in range(8):
            if i==0:
                T_list.append(np.matmul(np.identity(4), A_list[0]))
            else:
                T_list.append(np.matmul(T_list[i-1], A_list[i]))
        return T_list

    def position_correction(self, T_list):
        """
        correct the position calculated by DH convention
        INPUT:
        T_list: list of homogenous transformation matrixes of each joint with respect to the world frame
        OUTPUT:
        jointPositions: (8,3) actual joint positions
        """
        jointPositions = np.zeros((8,3))
        for i in range(8):
            jointPositions[i,:] = np.matmul(T_list[i],self.joint_bias[i])[0:3]
        return jointPositions

    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the world frame
        """
        q_list = np.concatenate(([0],q))
        q_list = q_list + self.q_bias

        A_list = self.compute_Ai(q_list)    
        T_list = self.compute_Hi(A_list)

        for i in range(7):
            if i == 0:
                axis_of_rotation_list = T_list[i][:3, 2]   # column z
            else:
                axis_of_rotation_list = np.vstack([axis_of_rotation_list, T_list[i][:3, 2]])   # column z
        
        axis_of_rotation_list = axis_of_rotation_list.T
        return axis_of_rotation_list
    
    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    configurations = [
    np.array([ 0,    0,     0, -pi/2,     0, pi/2, pi/4 ]),
    np.array([ pi/2,    -pi/2,     pi/2, -pi/2,     0, pi/2, pi/4 ]),
    np.array([ pi/2, 0,  pi/4, -pi/2, -pi/2, pi/2,    0 ]),
    ]

    for i in range(3):
        joint_positions, T0e = fk.forward(configurations[i])
        print("Configuration",i+1)
        print("Joint Positions:\n",joint_positions)
        print("End Effector Pose:\n",T0e)
