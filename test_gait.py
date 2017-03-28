import numpy as num
import numpy.matlib
from gait import gait
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import mpl_toolkits.mplot3d.art3d as art3d

num.set_printoptions(precision=5)
wstep = 0.05
whip = 0.114
gait_param = num.array([[0, wstep, wstep, wstep, 0],
                        [whip, whip, whip, whip, whip]])
UXA_gait = gait(gait_param)
zmp_ref, zmp_time, rankle_pos, rankle_vel, lankle_pos, lankle_vel, com_pos, com_vel, zmp_out = UXA_gait.gait_generation(0.6, 1.0)
# Plot gait
plt.subplot(2, 1, 1)
plt.plot(zmp_time, zmp_out[:, 0], 'r-')
plt.plot(zmp_time, zmp_ref[:, 0], 'b--')
plt.plot(zmp_time, com_pos[:, 0], 'g-')
plt.title('Walking gait 4 step')
plt.ylabel('x(m)')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(zmp_time, zmp_out[:, 1], 'r-')
plt.plot(zmp_time, zmp_ref[:, 1], 'b--')
plt.plot(zmp_time, com_pos[:, 1], 'g-')
plt.xlabel('time (s)')
plt.ylabel('y(m)')
plt.grid(True)
plt.show()