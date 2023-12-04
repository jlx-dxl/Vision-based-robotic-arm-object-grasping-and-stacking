import numpy as np
from lib.calculateFK import FK

def calcJacobian(q):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    Jacobian = np.zeros((6, 7))

    fk = FK()

    joint_positions, _ = fk.forward(q)
    axis_of_rotations = fk.get_axis_of_rotation(q)
    o_n = joint_positions[-1, :]

    for i in range(7):
        r = np.array(o_n - joint_positions[i, :])
        Jacobian[:3, i] = np.cross(axis_of_rotations[:, i], r)   # linear velocity
        Jacobian[3:, i] = axis_of_rotations[:, i]   # angular velocity

    return Jacobian


if __name__ == '__main__':
    # q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    q= np.array([0, 0, 0, 0, 0, 0, 0])
    print(np.round(calcJacobian(q),3))
