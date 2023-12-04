from lib.calculateFK import FK
from core.interfaces import ArmController
import numpy as np
from tqdm import tqdm

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
ax = fig.add_subplot(232, projection='3d')
x = fig.add_subplot(234)
y = fig.add_subplot(235)
z = fig.add_subplot(236)

# TODO: update this with real results
resolution = 100
random = 50000
qs = np.zeros((7,resolution))
positions = np.zeros((3,random))

for i in range(7):
    qs[i,:] = np.array(np.linspace(limits[i]['lower'],limits[i]['upper'],resolution))

for j in tqdm(range(random)):
    random_idx = np.random.choice(qs.shape[1],size=(7,1)).reshape(-1)
    # print(random_idx,random_idx.shape)
    random_q = qs[np.arange(7),random_idx]
    # print(random_q,random_q.shape)
    joint_posistions, _ = fk.forward(random_q)
    positions[:,j] = joint_posistions[-1,:]

size = 0.5
ax.scatter(positions[0,:],positions[1,:],positions[2,:],s=size)
ax.set_title("3D")
x.scatter(positions[1,:],positions[2,:],s=size)
x.set_title("x axis")
y.scatter(positions[0,:],positions[2,:],s=size)
y.set_title("y axis")
z.scatter(positions[0,:],positions[1,:],s=size)
z.set_title("z axis")

plt.show()

# plt.show()
