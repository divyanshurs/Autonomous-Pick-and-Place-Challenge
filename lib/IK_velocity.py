import numpy as np
from lib.calcJacobian import calcJacobian


def IK_velocity(q_in, v_in, omega_in):
    """
    :param q: 0 x 7 vector corresponding to the robot's current configuration.
    :param v: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 0 x 7 vector corresponding to the joint velocities. If v and omega
         are infeasible, then dq should minimize the least squares error. If v
         and omega have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """

    ## STUDENT CODE GOES HERE
    #print(v_in)
    #print(omega_in)
    vel_mat = np.hstack((v_in, omega_in))
    # print(vel_mat)
    vel_mat = np.transpose(vel_mat)
    # print(vel_mat)
    bool=~np.isnan(vel_mat)
    vel_mat = vel_mat[~np.isnan(vel_mat)]
    jacobian = calcJacobian(q_in)
    jacobian = jacobian[bool]

    dq = np.zeros(7)

    arr = np.linalg.lstsq(jacobian, vel_mat)
    dq=arr[0]
    return dq
