from lib.calculateFK import FK
from core.interfaces import ArmController
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#from itertools import combinations

fk = FK()

# the dictionary below contains the data returned by calling arm.joint_limits()
limits = [
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -1.7628, 'upper': 1.7628},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -3.0718, 'upper': -0.0698},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -0.0175, 'upper': 3.7525},
    {'lower': -2.8973, 'upper': 2.8973}
 ]

# TODO: create plot(s) which visualize the reachable workspace of the Panda arm,
# accounting for the joint limits.
#
# We've included some very basic plotting commands below, but you can find
# more functionality at https://matplotlib.org/stable/index.html

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

j1 = np.linspace(limits[0]['lower'], limits[0]['upper'], 33)
j2 = np.linspace(limits[1]['lower'], limits[1]['upper'], 33)
j3 = np.linspace(limits[2]['lower'], limits[2]['upper'], 3)
j4 = np.linspace(limits[3]['lower'], limits[3]['upper'], 3)
j5 = np.linspace(limits[4]['lower'], limits[4]['upper'], 3)
j6 = np.linspace(limits[5]['lower'], limits[5]['upper'], 3)
j7 = np.linspace(limits[6]['lower'], limits[6]['upper'], 10)


total_list = [j1,j2,j3,j4,j5,j6,j7]
temp_list = []

Positions= []
count = 0
for i in j1:
    for j in j2:
        temp_list = [i, j, limits[2]['upper'], limits[3]['upper'], limits[4]['upper'], limits[5]['upper'], limits[6]['upper']] #outer reachable workspace
        temp_list_2 = [i, j, limits[2]['lower'], limits[3]['lower'], limits[4]['lower'], limits[5]['lower'], limits[6]['lower']] #inner reachable workspace
        _, end_effector_matrix = fk.forward(temp_list)
        _, end_effector_matrix_2 = fk.forward(temp_list_2)
        position_mat = end_effector_matrix[:-1, 3]
        position_mat_2 = end_effector_matrix_2[:-1, 3]
        ax.scatter(position_mat[0], position_mat[1], position_mat[2])
        ax.scatter(position_mat_2[0], position_mat_2[1], position_mat_2[2])


plt.show()
