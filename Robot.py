from Link import Link
import numpy as num
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import mpl_toolkits.mplot3d.art3d as art3d
import matplotlib.pyplot as plt
from scipy.linalg import norm
from numpy.linalg import inv


class Robot(Link):
    """Class Link

        Longer class information....
        Longer class information....

        Attributes:
            Attributes 1: information....
            Attributes 2: information....
    """
    # TODO(nvtienanh): Adding more comment.

    def __init__(self):
        """Inits Robot class"""
        super(Robot, self).__init__(_name=None, _id=None, _mass=None, _sister=None, _child=None)
        self.links = []
        # self.CoM = 0
        # self.ZMP

    def add_link(self, link):
        """Adding link to robot model."""
        self.links.append(link)

    def display_info(self):
        """Display robot links info."""
        for index in range(len(self.links)):
            self.links[index].info()

    def find_mother(self, idx):
        """Find mother of each robot link"""
        if idx is not None:
            if idx == 0:
                self.links[idx].mother = None

            if self.links[idx].child != None:
                child_id = self.links[idx].child
                self.links[child_id].mother = idx
                self.find_mother(child_id)

            if self.links[idx].sister != None:
                sister_id = self.links[idx].sister
                self.links[sister_id].mother = self.links[idx].mother
                self.find_mother(sister_id)

    def setup_rigid_body(self, id, shape, com):
        """Set up Rigid Body for simulation"""
        vert = num.array([[0, 0, 0],
                          [0, shape[1], 0],
                          [shape[0], shape[1], 0],
                          [shape[0], 0, 0],
                          [0, 0, shape[2]],
                          [0, shape[1], shape[2]],
                          [shape[0], shape[1], shape[2]],
                          [shape[0], 0, shape[2]]])
        vert = vert.T

        vert[0, :] = vert[0, :] - shape[0] / 2 + com[0]
        vert[1, :] = vert[1, :] - shape[1] / 2 + com[1]
        vert[2, :] = vert[2, :] - shape[2] / 2 + com[2]

        face = num.array([[0, 1, 2, 3],
                           [1, 5, 6, 2],
                           [3, 2, 6, 7],
                           [0, 4, 7, 3],
                           [0, 1, 5, 4],
                           [4, 5, 6, 7]])
        face = face.T
        inertia = num.array([[1/12*(shape[1]**2 + shape[2]**2), 0, 0],
                             [0, 1/12*(shape[0]**2 + shape[2]**2), 0],
                             [0, 0, 1/12*(shape[1]**2 + shape[2]**2)]])*self.links[id].m

        self.links[id].vertex = vert
        self.links[id].face = face
        self.links[id].c = com
        self.links[id].I = inertia

    def init_rigid_body(self):
        """ Setup rigid body for simulation """

        shape = num.array([0.08, 0.16, 0.52])  # Depth, width, height [m]
        com = num.array([[0], [0], [0.28]])    # Position of the center of gravity
        self.setup_rigid_body(0, shape, com)

        """ Setup rigid of RLEG_J5 """
        shape = num.array([0.255, 0.1, 0.03])
        com = num.array([[0.0325], [0], [-0.055]])
        self.setup_rigid_body(6, shape, com)

        """ Setup rigid of LLEG_J5 """
        self.setup_rigid_body(12, shape, com)

    def dh_to_trans_mat(self, dh_i, q_i):
        """ Calculate Transform matrix from DH table"""
        a_i = dh_i[0]
        alpha_i = dh_i[1]
        d_i = dh_i[2]
        offset_i = dh_i[3]
        q_i = q_i + offset_i

        T = num.array([[num.cos(q_i), -num.sin(q_i), 0, a_i],
                       [num.cos(alpha_i)*num.sin(q_i), num.cos(alpha_i)*num.cos(q_i), -num.sin(alpha_i), -d_i*num.sin(alpha_i)],
                       [num.sin(alpha_i)*num.sin(q_i), num.sin(alpha_i)*num.cos(q_i), num.cos(alpha_i), d_i*num.cos(alpha_i)],
                       [0, 0, 0, 1]])
        return T

    def forward_kinematic(self, id):
        """ Forward Kinematic robot"""
        if id is not None:
            if id != 0:
                mom = self.links[id].mother
                T_i = self.dh_to_trans_mat(self.links[id].dh, self.links[id].q)
                self.links[id].T = num.dot(self.links[mom].T, T_i)
                self.links[id].p = self.links[id].T[0:3][:, 3:4]
                self.links[id].R = self.links[id].T[0:3][:, 0:3]

            self.forward_kinematic(self.links[id].sister)
            self.forward_kinematic(self.links[id].child)

    def find_route(self, lk_id, idx = []):
        """
        Return the list of joint number connecting from Root to link_id lk_id
        :param lk_id:
        :return: array list of joint
        """
        ID = self.links[lk_id].mother
        if ID == 0:
            idx.append(lk_id)
        else:
            idx.append(lk_id)
            self.find_route(ID)

        return idx


    def cal_jacobian(self, chain):
        """
        Function calculate jacobian matrix
        :param chain: list of link
        :return: Jacobian matrix :6x6
        """
        j_size = len(chain)
        target = (self.links[chain[-1]]).p
        J = num.zeros((6, j_size))
        for index in range(j_size):
            j = chain[index]
            a = self.links[j].T[0:3][:, 2:3]
            J[:, index] = num.append(num.cross(a, target - self.links[j].p, axis=0), a)
        return J

    def cal_total_mass(self, lk_id):
        """
        Function calculate total of mass
        :param lk_id:
        :return: Total of mass
        """
        if lk_id is None:
            m = 0
        else:
            m = self.links[lk_id].m + self.cal_total_mass(self.links[lk_id].sister) + self.cal_total_mass(self.links[lk_id].child)

        return m

    def inverse_kinematic(self, pos_ref: Link, pos_now: Link):
        """
        Function calculate posture error
        :param pos_ref:
        :param pos_now:
        :return:
        """
        chain = self.find_route(pos_now.id)
        chain = chain[::-1]  # Reverse order
        #  Default value
        treshold = 10**-12
        max_iter = 50
        lamda_max = 0.09  # Variable damping
        beta = 0.001  # Isotropic damping
        epsilon = 0.001  # Singular region size

        iter_taken = 1
        dofs = len(chain)

        while True:
            iter_taken = iter_taken + 1
            jac = self.cal_jacobian(chain)
            posture_error = self.cal_posture_error(pos_ref, pos_now)
            norm_err = num.linalg.norm(posture_error)
            U, s, V = num.linalg.svd(jac, full_matrices=True)
            sigma_min = s[-1]
            u_m = U[:, 0:dofs][:, dofs-1:dofs]  # get column dofs-1 of matrix U
            lamda = lamda_max
            if sigma_min < epsilon:
                lamda = (1 - (sigma_min/epsilon)**2)*(lamda_max**2)
            jac_inv = num.dot(jac.T, num.linalg.inv(num.dot(jac, jac.T) + lamda ** 2 * num.eye(dofs) + beta ** 2 * num.dot(u_m, u_m.T)))
            dq = num.dot(jac_inv, posture_error)
            dq = dq.ravel()

            self.move_joints(chain, dq)
            self.forward_kinematic(0)

            if (iter_taken > max_iter) or (norm_err < treshold):
                break

    def cal_posture_error(self, pos_ref: Link, pos_now: Link):
        """
        Function calculate posture error
        :param pos_ref:
        :param pos_now:
        :return:
        """
        posture_err = num.zeros((6, 1))
        pos_err = pos_ref.p - pos_now.p

        ori_err = num.dot(num.linalg.inv(pos_now.R), pos_ref.R)
        ome_err = num.dot(pos_now.R, self.rot2omega(ori_err))
        posture_err[:, 0] = num.append(pos_err, ome_err)
        return posture_err

    def rot2omega(self, r_mat):
        """
        Function convert rotate matrix to omega
        :param r_mat:
        :return: omega
        """
        alpha = (num.trace(r_mat)-1)/2
        if num.abs(alpha - 1) < num.finfo(float).eps:
            w = num.zeros((3, 1))
            return w
        th = num.arccos(alpha)
        w = 0.5*th/num.sin(th)*num.array([[r_mat[2, 1] - r_mat[1, 2]],
                                [r_mat[0, 2] - r_mat[2, 0]],
                                [r_mat[1, 0] - r_mat[0, 1]]])
        return w

    def move_joints(self, chain, jnt_angle):
        """
        Function move joint
        :param chain:
        :param jnt_angle:
        :return:
        """
        for index in range(len(chain)):
            j = chain[index]
            self.links[j].q = self.links[j].q + jnt_angle[index]
            self.links[j].q = num.minimum(self.links[j].ub, num.maximum(self.links[j].q, self.links[j].lb))


    """PLOT FUCNTION LIST"""

    def draw_polygon(self, vert, face, col, axis):
        """
        Function drawing 3D polygon
        :param vert:
        :param face:
        :param col:
        :param axis:
        :return:
        """
        polys = []
        face = face.T  # Transpose matrix to match the rules
        vert = vert.T  # Transpose matrix to match the rules

        for index in range(6):
            """4 row of vertex
               match data of function patch https://www.mathworks.com/help/matlab/visualize/multifaceted-patches.html
            """
            vt_row_1 = face[index][0]
            vt_row_2 = face[index][1]
            vt_row_3 = face[index][2]
            vt_row_4 = face[index][3]
            x_i = [vert[vt_row_1][0], vert[vt_row_2][0], vert[vt_row_3][0], vert[vt_row_4][0]]
            y_i = [vert[vt_row_1][1], vert[vt_row_2][1], vert[vt_row_3][1], vert[vt_row_4][1]]
            z_i = [vert[vt_row_1][2], vert[vt_row_2][2], vert[vt_row_3][2], vert[vt_row_4][2]]
            polys.append(list(zip(x_i, y_i, z_i)))

        axis.add_collection3d(Poly3DCollection(polys, edgecolors='k', facecolors=col))

    def draw_cylinder(self, pos, az, radius, length, col, axis):
        """
        Drawing Cylinder
        :param pos:
        :param az:
        :param radius:
        :param length:
        :param col:
        :param axis:
        :return:
        """
        p0 = pos - az * length / 2
        p1 = pos + az * length / 2
        p0 = p0.ravel()
        p1 = p1.ravel()
        # vector in direction of axis
        v = p1 - p0

        # find magnitude of vector
        mag = norm(v)

        # unit vector in direction of axis
        v = v / mag

        # make some vector not in the same direction as v
        not_v = num.array([1, 0, 0])
        # make vector perpendicular to v
        n1 = num.cross(v, not_v)
        if norm(n1) < 10**-9:
            not_v = num.array([0, 1, 0])
        n1 = num.cross(v, not_v)

        # normalize n1

        n1 = n1/norm(n1)

        # make unit vector perpendicular to v and n1
        n2 = num.cross(v, n1)

        # surface ranges over t from 0 to length of axis and 0 to 2*pi
        t = num.linspace(0, mag, 2)
        theta = num.linspace(0, 2 * num.pi, 100)
        rsample = num.linspace(0, radius, 2)

        # use meshgrid to make 2d arrays
        t, theta2 = num.meshgrid(t, theta)
        rsample, theta = num.meshgrid(rsample, theta)
        # generate coordinates for surface
        # "Tube"
        X, Y, Z = [p0[i] + v[i] * t + radius * num.sin(theta2) * n1[i] + radius * num.cos(theta2) * n2[i] for i in [0, 1, 2]]
        # "Bottom"
        X2, Y2, Z2 = [p0[i] + rsample[i] * num.sin(theta) * n1[i] + rsample[i] * num.cos(theta) * n2[i] for i in [0, 1, 2]]
        # "Top"
        X3, Y3, Z3 = [p0[i] + v[i] * mag + rsample[i] * num.sin(theta) * n1[i] + rsample[i] * num.cos(theta) * n2[i] for i in [0, 1, 2]]

        axis.plot_surface(X, Y, Z, color=col)
        axis.plot_surface(X2, Y2, Z2, color=col)
        axis.plot_surface(X3, Y3, Z3, color=col)

    def connect_link(self, link1, link2, col, lw, axis):
        """
        Draw link connection
        :param link1: position of link 1
        :param link2: position of link 2
        :param col: color
        :param lw: line width
        :param axis: axis object
        :return: None
        """
        axis.plot([link1[0], link2[0]], [link1[1], link2[1]], zs=[link1[2], link2[2]], linewidth=lw, color=col)


    def draw_all_joint(self, id, axis):
        """
        Function draw all robot model
        :param id:
        :return:
        """
        radius = 0.018
        length = 0.06
        joint_col = 0
        if id is not None:
            if self.links[id].vertex is not None:

                ax_al_mat = num.dot(self.links[id].T[0:3][:, 0:3], inv(self.links[id].rot))
                vert = num.dot(ax_al_mat, self.links[id].vertex)
                for index in range(3):
                    vert[index, :] = vert[index, :] + self.links[id].p[index]

                self.draw_polygon(vert, self.links[id].face, 'g', axis)

            mom = self.links[id].mother
            if mom is not None:
                self.connect_link(self.links[mom].p, self.links[id].p, 'k', 2, axis)

            ax_al_mat = num.dot(self.links[id].T[0:3][:, 0:3], inv(self.links[id].rot))
            self.draw_cylinder(self.links[id].p, num.dot(ax_al_mat, self.links[id].a), radius, length, 'g',axis)
            self.draw_all_joint(self.links[id].child, axis)
            self.draw_all_joint(self.links[id].sister, axis)


