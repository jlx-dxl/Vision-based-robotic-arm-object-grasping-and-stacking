import numpy as np
from lib.calculateFK import FK


def cal_Jacobian(q):
    """
    input: q - 1 x 7 configurantion(joint angles)
    output: Jacobian - 6 x 7 Jacobian matrix
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


def FK_Jacobian(q, dq):
    """
    input:  q - 1 x 7 configurantion(joint angles)
            dq - 1 x 7 joint velocities
    output: v_w - 6 x 1 linear_velocities and angular_velocities
    """

    Jacobian = cal_Jacobian(q)
    # dq = dq.reshape((7,1))
    v_w = np.matmul(Jacobian,dq)

    return v_w


if __name__ == '__main__':

    q= np.array([0, 0, 0, 0, 0, 0, 0])
    print('Jacobian Matrix at 0 configuration:\n',np.round(cal_Jacobian(q),6))

    for i in range(7):
        dq = np.zeros((1,7)).reshape(-1)
        dq[i]=1
        print('dq:\n',dq)

        v_w = FK_Jacobian(q, dq)
        linear_velocity = v_w[:3]
        angular_velocity = v_w[3:]
        print('linear_velocity:\n', np.round(linear_velocity,6))
        print('angular_velocity:\n', np.round(angular_velocity,6))
