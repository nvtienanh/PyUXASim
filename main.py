import numpy as num
from Link import Link
from Robot import Robot
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import mpl_toolkits.mplot3d.art3d as art3d

"""
Init robot structure

"""

UXA = Robot()

UX = num.array([[1], [0], [0]])
UY = num.array([[0], [1], [0]])
UZ = num.array([[0], [0], [1]])

BODY = Link('BODY', 0, 3.5, None, 1)
BODY.dh = num.array([0, 0, 0, 0])
BODY.rot = num.array([[0, -1, 0],
                      [1, 0, 0],
                      [0, 0, 1]])
BODY.a = UZ
UXA.add_link(BODY)

RLEG_J0 = Link('RLEG_J0', 1, 0.5, 7, 2)
RLEG_J0.dh = num.array([-0.057, 0, -0.042, 0])
RLEG_J0.rot = num.array([[0, -1, 0],
                         [1, 0, 0],
                         [0, 0, 1]])
RLEG_J0.a = UZ
UXA.add_link(RLEG_J0)

RLEG_J1 = Link('RLEG_J1', 2, 0.5, None, 3)
RLEG_J1.dh = num.array([0, num.pi/2, 0, -num.pi/2])
RLEG_J1.rot = num.array([[0, 0, 1],
                         [0, 1, 0],
                         [-1, 0, 0]])
RLEG_J1.a = UX
UXA.add_link(RLEG_J1)

RLEG_J2 = Link('RLEG_J2', 3, 0.5, None, 4)
RLEG_J2.dh = num.array([0, -num.pi/2, 0, 0])
RLEG_J2.rot = num.array([[0, -1, 0],
                         [0, 0, 1],
                         [-1, 0, 0]])
RLEG_J2.a = UY
UXA.add_link(RLEG_J2)

RLEG_J3 = Link('RLEG_J3', 4, 0.5, None, 5)
RLEG_J3.dh = num.array([0.21, 0, 0, 0])
RLEG_J3.rot = num.array([[0, -1, 0],
                         [0, 0, 1],
                         [-1, 0, 0]])
RLEG_J3.a = UY
RLEG_J3.lb = 0
UXA.add_link(RLEG_J3)

RLEG_J4 = Link('RLEG_J4', 5, 0.5, None, 6)
RLEG_J4.dh = num.array([0.21, 0, 0, 0])
RLEG_J4.rot = num.array([[0, -1, 0],
                         [0, 0, 1],
                         [-1, 0, 0]])
RLEG_J4.a = UY
UXA.add_link(RLEG_J4)

RLEG_J5 = Link('RLEG_J5', 6, 0.5, None, None)
RLEG_J5.dh = num.array([0, num.pi/2, 0, 0])
RLEG_J5.rot = num.array([[0, 0, 1],
                         [0, 1, 0],
                         [-1, 0, 0]])
RLEG_J5.a = UX
UXA.add_link(RLEG_J5)

LLEG_J0 = Link('LLEG_J0', 7, 0.5, None, 8)
LLEG_J0.dh = num.array([0.057, 0, -0.042, 0])
LLEG_J0.rot = num.array([[0, -1, 0],
                         [1, 0, 0],
                         [0, 0, 1]])
LLEG_J0.a = UZ
UXA.add_link(LLEG_J0)

LLEG_J1 = Link('LLEG_J1', 8, 0.5, None, 9)
LLEG_J1.dh = num.array([0, num.pi/2, 0, -num.pi/2])
LLEG_J1.rot = num.array([[0, 0, 1],
                         [0, 1, 0],
                         [-1, 0, 0]])
LLEG_J1.a = UX
UXA.add_link(LLEG_J1)

LLEG_J2 = Link('LLEG_J2', 9, 0.5, None, 10)
LLEG_J2.dh = num.array([0, -num.pi/2, 0, 0])
LLEG_J2.rot = num.array([[0, -1, 0],
                         [0, 0, 1],
                         [-1, 0, 0]])
LLEG_J2.a = UY
UXA.add_link(LLEG_J2)

LLEG_J3 = Link('LLEG_J3', 10, 0.5, None, 11)
LLEG_J3.dh = num.array([0.21, 0, 0, 0])
LLEG_J3.rot = num.array([[0, -1, 0],
                         [0, 0, 1],
                         [-1, 0, 0]])
LLEG_J3.a = UY
LLEG_J3.lb = 0
UXA.add_link(LLEG_J3)

LLEG_J4 = Link('LLEG_J4', 11, 0.5, None, 12)
LLEG_J4.dh = num.array([0.21, 0, 0, 0])
LLEG_J4.rot = num.array([[0, -1, 0],
                         [0, 0, 1],
                         [-1, 0, 0]])
LLEG_J4.a = UY
UXA.add_link(LLEG_J4)

LLEG_J5 = Link('LLEG_J5', 12, 0.5, None, None)
LLEG_J5.dh = num.array([0, num.pi/2, 0, 0])
LLEG_J5.rot = num.array([[0, 0, 1],
                         [0, 1, 0],
                         [-1, 0, 0]])
LLEG_J5.a = UX
UXA.add_link(LLEG_J5)

UXA.find_mother(0)
UXA.init_rigid_body()

UXA.links[0].T = num.array([[0, -1, 0, 0],
                            [1, 0, 0, 0],
                            [0, 0, 1, 0.462],
                            [0, 0, 0, 1]])
UXA.links[0].p = UXA.links[0].T[0:3][:, 3:4]
UXA.links[0].R = UXA.links[0].T[0:3][:, 0:3]
UXA.forward_kinematic(0)
#  Finish init robot structure

#  Adding desired posture of left Foot and right Foot object
RFoot = Link('RFoot', 0, 0, None, 0)
LFoot = Link('LFoot', 0, 0, None, 0)
RFoot.p = num.array([[0], [-0.057], [0.1]])
LFoot.p = num.array([[0], [0.057], [0.1]])
RFoot.R = UXA.links[6].R
LFoot.R = UXA.links[12].R

# Inverse kinematic
UXA.inverse_kinematic(RFoot, UXA.links[6])

UXA.forward_kinematic(0)

fig = plt.figure()
ax = Axes3D(fig)
ax.set_aspect('equal')

max_range = 0.5
mid_x = 0.15
mid_y = 0
mid_z = 0.5

ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

UXA.draw_all_joint(0, ax)

route = UXA.find_route(5)
route = route[::-1]
# print(UXA.cal_posture_error(RFoot, UXA.links[6]))

plt.show()
