import numpy as np 
from lib.calcJacobian import calcJacobian

def FK_velocity(q_in, dq):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param dq: 1 x 7 vector corresponding to the joint velocities.
    :return:
    velocity - 6 x 1 vector corresponding to the end effector velocities.    
    """

    ## STUDENT CODE GOES HERE

    Jacobian = calcJacobian(q_in)
    dq = dq.reshape((7,1))
    v_w = np.matmul(Jacobian,dq)

    return v_w

if __name__ == '__main__':
    # q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    q= np.zeros((7,))

    for i in range(7):
        qdot = np.zeros(7,)
        qdot[i] = 1
        velocity = FK_velocity(q,qdot)
        print("moving the joint" + str(i+1) + ", the ee velocity is:", np.round(velocity.T,3))
