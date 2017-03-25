import numpy as num


class Link(object):
    """Class Link

        Longer class information....
        Longer class information....

        Attributes:
            Attributes 1: information....
            Attributes 2: information....
    """
    # TODO(nvtienanh): Adding more comment.

    def __init__(self, _name, _id, _mass, _sister, _child):
        self.mother = None           # Mother ID
        self.name = _name            # Name of this link
        self.id = _id                # ID of this link
        self.m = _mass               # Mass of this link
        self.sister = _sister        # Sister ID of this link
        self.child = _child          # Child ID of this link
        self.T = num.eye(4)          # Transformation matrix of this link
        self.ub = num.pi/2           # Upper boundary of link angle
        self.lb = -num.pi/2          # Lower boundary of link angle
        self.dh = num.zeros((1, 4))  # DH parameter of this link
        self.q = 0                   # Angle of link joint
        self.dq = 0                  # Joint velocity  [rad/s]
        self.ddq = 0                 # Joint acceleration [rad/s^2]
        self.c = num.zeros((3, 1))   # Position of the center of gravity [m]
        self.I = num.zeros((3, 3))   # Inertia tensor of the center of gravity around [kg.m^2]
        self.p = num.zeros((3, 1))   # Position of this link
        self.R = num.zeros((3, 1))   # Orientation of this link
        self.rot = None              # this argument using to convert self.R to axis angle
        self.a = None                # Axis angle of link

        """Data for 3D simulation"""
        self.vertex = None
        self.face = None

    def info(self):
        print(self.mother)

