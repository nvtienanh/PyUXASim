import numpy as num
from Link import Link
from Robot import Robot
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import mpl_toolkits.mplot3d.art3d as art3d

'''(self, Name, Id, Mass, Sister, Child)'''
# The list of biped robot link
UXA = Robot()
Links = []

UX = num.array([[1], [0], [0]])
UY = num.array([[0], [1], [0]])
UZ = num.array([[0], [0], [1]])

BODY = Link('BODY', 0, 3.5, None, 1)
BODY.dh = num.array([0, 0, 0, 0])
BODY.rot = num.array([[0, -1, 0],
                      [1, 0, 0],
                      [0, 0, 1]])
BODY.a = UZ
Links.append(BODY)
UXA.add_link(BODY)

RLEG_J0 = Link('RLEG_J0', 1, 0.5, 7, 2)
RLEG_J0.dh = num.array([-0.057, 0, -0.042, 0])
RLEG_J0.rot = num.array([[0, -1, 0],
                         [1, 0, 0],
                         [0, 0, 1]])
RLEG_J0.a = UZ
Links.append(RLEG_J0)
UXA.add_link(RLEG_J0)

RLEG_J1 = Link('RLEG_J1', 2, 0.5, None, 3)
RLEG_J1.dh = num.array([0, num.pi/2, 0, -num.pi/2])
RLEG_J1.rot = num.array([[0, 0, 1],
                         [0, 1, 0],
                         [-1, 0, 0]])
RLEG_J1.a = UX
Links.append(RLEG_J1)
UXA.add_link(RLEG_J1)

RLEG_J2 = Link('RLEG_J2', 3, 0.5, None, 4)
RLEG_J2.dh = num.array([0, -num.pi/2, 0, 0])
RLEG_J2.rot = num.array([[0, -1, 0],
                         [0, 0, 1],
                         [-1, 0, 0]])
RLEG_J2.a = UY
Links.append(RLEG_J2)
UXA.add_link(RLEG_J2)

RLEG_J3 = Link('RLEG_J3', 4, 0.5, None, 5)
RLEG_J3.dh = num.array([0.21, 0, 0, 0])
RLEG_J3.rot = num.array([[0, -1, 0],
                         [0, 0, 1],
                         [-1, 0, 0]])
RLEG_J3.a = UY
Links.append(RLEG_J3)
UXA.add_link(RLEG_J3)

RLEG_J4 = Link('RLEG_J4', 5, 0.5, None, 6)
RLEG_J4.dh = num.array([0.21, 0, 0, 0])
RLEG_J4.rot = num.array([[0, -1, 0],
                         [0, 0, 1],
                         [-1, 0, 0]])
RLEG_J4.a = UY
Links.append(RLEG_J4)
UXA.add_link(RLEG_J4)

RLEG_J5 = Link('RLEG_J5', 6, 0.5, None, None)
RLEG_J5.dh = num.array([0, num.pi/2, 0, 0])
RLEG_J5.rot = num.array([[0, 0, 1],
                         [0, 1, 0],
                         [-1, 0, 0]])
RLEG_J5.a = UX
Links.append(RLEG_J5)
UXA.add_link(RLEG_J5)

LLEG_J0 = Link('LLEG_J0', 7, 0.5, None, 8)
LLEG_J0.dh = num.array([0.057, 0, -0.042, 0])
LLEG_J0.rot = num.array([[0, -1, 0],
                         [1, 0, 0],
                         [0, 0, 1]])
LLEG_J0.a = UZ
Links.append(LLEG_J0)
UXA.add_link(LLEG_J0)

LLEG_J1 = Link('LLEG_J1', 8, 0.5, None, 9)
LLEG_J1.dh = num.array([0, num.pi/2, 0, -num.pi/2])
LLEG_J1.rot = num.array([[0, 0, 1],
                         [0, 1, 0],
                         [-1, 0, 0]])
LLEG_J1.a = UX
Links.append(LLEG_J1)
UXA.add_link(LLEG_J1)

LLEG_J2 = Link('LLEG_J2', 9, 0.5, None, 10)
LLEG_J2.dh = num.array([0, -num.pi/2, 0, 0])
LLEG_J2.rot = num.array([[0, -1, 0],
                         [0, 0, 1],
                         [-1, 0, 0]])
LLEG_J2.a = UY
Links.append(LLEG_J2)
UXA.add_link(LLEG_J2)

LLEG_J3 = Link('LLEG_J3', 10, 0.5, None, 11)
LLEG_J3.dh = num.array([0.21, 0, 0, 0])
LLEG_J3.rot = num.array([[0, -1, 0],
                         [0, 0, 1],
                         [-1, 0, 0]])
LLEG_J3.a = UY
Links.append(LLEG_J3)
UXA.add_link(LLEG_J3)

LLEG_J4 = Link('LLEG_J4', 11, 0.5, None, 12)
LLEG_J4.dh = num.array([0.21, 0, 0, 0])
LLEG_J4.rot = num.array([[0, -1, 0],
                         [0, 0, 1],
                         [-1, 0, 0]])
LLEG_J4.a = UY
Links.append(LLEG_J4)
UXA.add_link(LLEG_J4)

LLEG_J5 = Link('LLEG_J5', 12, 0.5, None, None)
LLEG_J5.dh = num.array([0, num.pi/2, 0, 0])
LLEG_J5.rot = num.array([[0, 0, 1],
                         [0, 1, 0],
                         [-1, 0, 0]])
LLEG_J5.a = UX
Links.append(LLEG_J5)
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

# ax = plt.subplot(111, projection='3d')
fig = plt.figure()
ax = Axes3D(fig)
UXA.draw_all_joint(0, ax)

print(UXA.find_route(5))
plt.show()