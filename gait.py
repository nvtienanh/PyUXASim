import numpy as num
import scipy.linalg
import numpy.matlib

class gait(object):
    """
    Class biped gait gernerator
    """
    def __init__(self, gait_param = None):
        self.Tcycle = 1.4
        self.DSrate = 0.2
        self.SSrate = 0.8
        self.SWrate = 0.4
        self.STrate = 0.6
        self.samp_per_sec = 100
        self.dt = 1.0/self.samp_per_sec
        self.HipWidth = 0.114
        self.stepHeight = 0.03

        self.gait_param = gait_param
        #  self.zmp_design = gait_param
        #  self.zmp_design[1, 0] = self.zmp_design[1, 0]/2
        #  self.zmp_design[1, -1] = self.zmp_design[1, -1]/2

        #  Output argument
        self.left_ankle = None
        self.right_ankle = None
        self.vel_left_ankle = None  # Left Ankle velocity
        self.vel_right_ankle = None  # Right Ankle velocity
        self.com = None
        self.vel_com = None
        self.zmp_ref = None
        self.zmp_out = None
        self.time = None

    def cal_stance_foot(self, init_stance, first_leg):
        """
        Fucntion calculate stance foot step planer
        :param foot_step:
        :param init_stance:
        :param first_leg:
        :return: Ankle position and velocity
        """
        s_x = self.gait_param[0, :]
        s_y = self.gait_param[1, :]
        row, col = self.gait_param.shape
        num_step = col
        # Which leg is stance first?
        if first_leg is 'Left':
            left_stance_first = False
        else:
            left_stance_first = True

        stance_foot = num.zeros((num_step+1, 2))
        stance_foot[0, 0] = init_stance[0]
        stance_foot[0, 1] = init_stance[1]

        for index in range(1, num_step+1):
            #  Equation 4.50 in text book: Introduction to humanoid robotics
            stance_foot[index, 0] = stance_foot[index-1, 0] + s_x[index-1]
            if left_stance_first:
                stance_foot[index, 1] = stance_foot[index-1, 1] - (-1)**index*s_y[index-1]
            else:
                stance_foot[index, 1] = stance_foot[index - 1, 1] + (-1)**index * s_y[index - 1]

        t_ds = self.Tcycle/2*self.DSrate
        t_sw = self.Tcycle*self.SWrate
        t_st = self.Tcycle*self.STrate
        ds_period = num.arange(0.0, t_ds+self.dt, self.dt, dtype=float)
        sw_period = num.arange(0.0, t_sw + self.dt, self.dt, dtype=float)
        st_period = num.arange(0.0, t_st + self.dt, self.dt, dtype=float)

        # Calculate number of stance phase of each foot
        left_foot_stance = None
        right_foot_stance = None

        if left_stance_first:
            left_flag = False
        else:
            left_flag = True

        for index in range(num_step+1):
            if left_flag:
                if left_foot_stance is None:
                    left_foot_stance = num.append(stance_foot[index, :], 0)
                else:
                    left_foot_stance = num.vstack((left_foot_stance, num.append(stance_foot[index, :], 0)))
            else:
                if right_foot_stance is None:
                    right_foot_stance = num.append(stance_foot[index, :], 0)
                else:
                    right_foot_stance = num.vstack((right_foot_stance, num.append(stance_foot[index, :], 0)))

            left_flag = not left_flag

        # When left leg is stance first
        if left_stance_first:
            #  Todo : add code in the case left leg is stace first. Maybe reverse the code in else case
            print('You need to add  code in this case!!')
        else:
            """ In this case: right leg is the first stance leg
            Calculate the position of foot in timeseries
            """
            rfoot_time = [0]
            rfoot_pos = right_foot_stance[0,:]
            row, col = right_foot_stance.shape
            for index in range(1, row):
                # Stance phase of right leg
                pre_time = rfoot_time[-1] # previous time
                rfoot_time = num.append(rfoot_time, pre_time+t_st)
                rfoot_pos = num.vstack((rfoot_pos, right_foot_stance[index-1, :]))
                # Stance phase of right leg
                pre_time = rfoot_time[-1]
                rfoot_time = num.append(rfoot_time, pre_time + t_sw)
                rfoot_pos = num.vstack((rfoot_pos, right_foot_stance[index, :]))

            # Left leg is swing
            lfoot_time = [0]
            lfoot_pos = None
            row, col = left_foot_stance.shape
            for index in range(1, row):
                pre_time = lfoot_time[-1]
                lfoot_time = num.append(lfoot_time, pre_time + t_sw)
                # Swing phase
                if lfoot_pos is None:
                    lfoot_pos = left_foot_stance[index - 1, :]
                else:
                    lfoot_pos = num.vstack((lfoot_pos, left_foot_stance[index - 1, :]))
                # Stance phase
                pre_time = lfoot_time[-1]
                lfoot_time = num.append(lfoot_time, pre_time + t_st)
                lfoot_pos = num.vstack((lfoot_pos, left_foot_stance[index, :]))
            # Adding the final state
            lfoot_pos = num.vstack((lfoot_pos, left_foot_stance[-1, :]))

            # Create Ankle data for Right leg first because this is the fist stance leg
            rankle_time = None
            rankle_pos = None
            rankle_vel = None
            pre_time = 0
            for index in range(len(rfoot_pos)-1):
                if rfoot_pos[index, 0] == rfoot_pos[index+1, 0]:
                    # Right leg is in the stance phase
                    if rankle_pos is None:
                        rankle_time = pre_time + st_period
                        rankle_pos = num.matlib.repmat(rfoot_pos[index, :], len(st_period), 1)
                        rankle_vel = num.matlib.repmat([0, 0, 0], len(st_period), 1)
                    else:
                        rankle_time = num.append(rankle_time, pre_time + st_period)
                        rankle_pos = num.vstack((rankle_pos, num.matlib.repmat(rfoot_pos[index, :], len(st_period), 1)))
                        rankle_vel = num.vstack((rankle_vel, num.matlib.repmat([0, 0, 0], len(st_period), 1)))
                    pre_time = rankle_time[-1]

                else:
                    # Right leg is in the swing phase
                    rankle_time = num.append(rankle_time, pre_time + sw_period)
                    x_pos, x_vel = self.interpolation_ankle_x(rfoot_pos[index, 0], rfoot_pos[index+1, 0], rfoot_time[index], sw_period)
                    z_pos, z_vel = self.interpolation_ankle_z(self.stepHeight, sw_period)
                    y_pos = num.matlib.repmat(rfoot_pos[index, 1], len(sw_period), 1)
                    y_vel = num.matlib.repmat(0, len(sw_period), 1)
                    rankle_pos = num.vstack((rankle_pos, num.hstack((x_pos.reshape(y_pos.shape), y_pos, z_pos.reshape(y_pos.shape)))))
                    rankle_vel = num.vstack((rankle_vel, num.hstack((x_vel.reshape(y_pos.shape), y_vel, z_vel.reshape(y_pos.shape)))))
                    pre_time = rankle_time[-1]

            # Create Ankle data for left leg
            lankle_time = None
            lankle_pos = None
            lankle_vel = None
            pre_time = 0
            for index in range(len(lfoot_pos)-1):
                if lfoot_pos[index, 0] == lfoot_pos[index + 1, 0]:
                    # Left foot is in stance phase
                    if lankle_pos is None:  # for store data at first step
                        lankle_time = pre_time + st_period
                        lankle_pos = num.matlib.repmat(lfoot_pos[index, :], len(st_period), 1)
                        lankle_vel = num.matlib.repmat([0, 0, 0], len(st_period), 1)
                    else:
                        lankle_time = num.append(lankle_time, pre_time + st_period)
                        lankle_pos = num.vstack((lankle_pos, num.matlib.repmat(lfoot_pos[index, :], len(st_period), 1)))
                        lankle_vel = num.vstack((lankle_vel, num.matlib.repmat([0, 0, 0], len(st_period), 1)))
                    pre_time = lankle_time[-1]
                else:
                    # Left leg is in the swing phase
                    if lankle_pos is None:  # for store data at first step
                        lankle_time = pre_time + sw_period
                        x_pos, x_vel = self.interpolation_ankle_x(lfoot_pos[index, 0], lfoot_pos[index + 1, 0],
                                                                  lfoot_time[index], sw_period)
                        z_pos, z_vel = self.interpolation_ankle_z(self.stepHeight, sw_period)
                        y_pos = num.matlib.repmat(lfoot_pos[index, 1], len(sw_period), 1)
                        y_vel = num.matlib.repmat(0, len(sw_period), 1)
                        lankle_pos = num.hstack((x_pos.reshape(y_pos.shape), y_pos, z_pos.reshape(y_pos.shape)))
                        lankle_vel = num.hstack((x_vel.reshape(y_pos.shape), y_vel, z_vel.reshape(y_pos.shape)))
                    else:
                        lankle_time = num.append(lankle_time, pre_time + sw_period)
                        x_pos, x_vel = self.interpolation_ankle_x(lfoot_pos[index, 0], lfoot_pos[index + 1, 0],
                                                                  lfoot_time[index], sw_period)
                        z_pos, z_vel = self.interpolation_ankle_z(self.stepHeight, sw_period)
                        y_pos = num.matlib.repmat(lfoot_pos[index, 1], len(sw_period), 1)
                        y_vel = num.matlib.repmat(0, len(sw_period), 1)
                        lankle_pos = num.vstack(
                            (lankle_pos, num.hstack((x_pos.reshape(y_pos.shape), y_pos, z_pos.reshape(y_pos.shape)))))
                        lankle_vel = num.vstack(
                            (lankle_vel, num.hstack((x_vel.reshape(y_pos.shape), y_vel, z_vel.reshape(y_pos.shape)))))
                    pre_time = lankle_time[-1]

            """ Vi tri ban dau hai chan o vi tri double support
                Vi then trong khoang thoi gian 0 ->T_ds, chan trai dung yen
                Bo sung them khoang thoi gian do vao chan trai
                Xem them hinh anh human walking giat (search google)
            """
            lankle_pos_init = num.matlib.repmat(left_foot_stance[0, :], len(ds_period), 1)
            lankle_vel_init = num.matlib.repmat([0, 0, 0], len(ds_period), 1)
            lankle_time_init = ds_period
            lankle_pos = num.vstack((lankle_pos_init, lankle_pos))
            lankle_vel = num.vstack((lankle_vel_init, lankle_vel))
            lankle_time = num.append(ds_period, lankle_time + ds_period[-1])

        return rankle_pos, rankle_vel, rankle_time, lankle_pos, lankle_vel, lankle_time

    def interpolation_ankle_x(self, x_start, x_end, t_start, t_sw):
        """
        Function interpolation ankle position in x direction
        :param x_start:
        :param x_end:
        :param t_start:
        :param t_sw:
        :return: Ankle position and velocity in x direction
        """
        t1 = 0.0
        t3 = t_sw[-1]
        t2 = (t1 + t3)/2
        f1 = 0
        f3 = x_end - x_start
        f2 = (f1 + f3)/2

        A = num.array([[t1**4, t1**3, t1**2, t1, 1],
                       [t2**4, t2**3, t2**2, t2, 1],
                       [t3**4, t3**3, t3**2, t3, 1],
                       [4*t1**3, 3*t1**2, 2*t1, 1, 0],
                       [4*t3**3, 3*t3**2, 2*t3, 1, 0]])
        Y = num.array([[f1],
                       [f2],
                       [f3],
                       [0],
                       [0]])
        X = num.dot(num.linalg.inv(A), Y)
        X = X.ravel()
        times = t_sw
        x_pos = x_start + self.poly4th(X, times)
        x_vel = self.poly4th_diff(X, times)
        return x_pos, x_vel

    def interpolation_ankle_z(self, step_height, t_sw):
        """
        Function interpolation ankle position in z direction
        :param step_height: Foot step height
        :param t_sw:
        :return: Ankle position and velocity in z direction
        """
        t1 = 0.0
        t3 = t_sw[-1]
        t2 = (t1 + t3)/2
        f1 = 0
        f2 = step_height
        f3 = 0

        A = num.array([[t1 ** 4, t1 ** 3, t1 ** 2, t1, 1],
                       [t2 ** 4, t2 ** 3, t2 ** 2, t2, 1],
                       [t3 ** 4, t3 ** 3, t3 ** 2, t3, 1],
                       [4 * t1 ** 3, 3 * t1 ** 2, 2 * t1, 1, 0],
                       [4 * t3 ** 3, 3 * t3 ** 2, 2 * t3, 1, 0]])
        Y = num.array([[f1],
                       [f2],
                       [f3],
                       [0],
                       [0]])
        X = num.dot(num.linalg.inv(A), Y)
        X = X.ravel()
        times = t_sw
        z_pos = self.poly4th(X, times)
        z_vel = self.poly4th_diff(X, times)
        return z_pos, z_vel

    def cal_zmp_ref(self, zmp_init, first_leg):
        """
        Function calculate the reference of ZMP
        :param zmp_init:
        :param first_leg:
        :return: zmp_ref, time
        """
        zmp_table = self.gait_param
        zmp_table[1, 0] = zmp_table[1, 0]/2
        zmp_table[1, -1] = zmp_table[1, -1] / 2
        zmp_raw_x = zmp_table[0, :]
        zmp_raw_y = zmp_table[1, :]

        row,col = zmp_table.shape
        # Which leg is stance first?
        if first_leg is 'Left':
            left_stance_first = False
        else:
            left_stance_first = True
        zmp_raw = num.zeros((col+1, 2))
        zmp_raw[0, 0] = zmp_init[0]
        zmp_raw[0, 1] = zmp_init[1]

        for index in range(1, col + 1):
            #  Equation 4.50 in text book: Introduction to humanoid robotics
            zmp_raw[index, 0] = zmp_raw[index - 1, 0] + zmp_raw_x[index - 1]
            if left_stance_first:
                zmp_raw[index, 1] = zmp_raw[index - 1, 1] - (-1) ** index * zmp_raw_y[index - 1]
            else:
                zmp_raw[index, 1] = zmp_raw[index - 1, 1] + (-1) ** index * zmp_raw_y[index - 1]
        # Init state before calculate
        t_ss = self.Tcycle/2*self.SSrate  # Single support duration
        t_ds = self.Tcycle/2*self.DSrate  # Double support duration
        ds_period = num.arange(0.0, t_ds + self.dt, self.dt, dtype=float)
        ds_num_sample = len(ds_period)
        ss_period = num.arange(0.0, t_ss + self.dt, self.dt, dtype=float)
        ss_num_sample = len(ss_period)

        time = None
        zmp_x = None
        zmp_y = None
        pre_time = 0
        for index in range(1, col+1):
            # Double support phase
            zmpx = num.linspace(zmp_raw[index - 1, 0], zmp_raw[index, 0], ds_num_sample)
            zmpy = num.linspace(zmp_raw[index - 1, 1], zmp_raw[index, 1], ds_num_sample)
            # Store data
            if time is None:
                # Store in the fist time
                time = pre_time + ds_period
                zmp_x = zmpx
                zmp_y = zmpy
            else:
                time = num.append(time, pre_time + ds_period)
                zmp_x = num.append(zmp_x, zmpx)
                zmp_y = num.append(zmp_y, zmpy)
            # Single support phase
            zmpx = num.matlib.repmat(zmp_raw[index, 0], ss_num_sample, 1)
            zmpy = num.matlib.repmat(zmp_raw[index, 1], ss_num_sample, 1)
            pre_time = time[-1]
            # Store data
            time = num.append(time, pre_time + ss_period)
            zmp_x = num.append(zmp_x, zmpx)
            zmp_y = num.append(zmp_y, zmpy)
            pre_time = time[-1]
        zmp_x = zmp_x.reshape((len(zmp_x), 1))
        zmp_y = zmp_y.reshape((len(zmp_y), 1))
        zmp_ref = num.hstack((zmp_x, zmp_y))

        return zmp_ref, time

    def poly4th(self, a, x):
        """
        Function calculate 4the poly function value
        :param a:
        :param x:
        :return:
        """
        value = a[0]*x**4 + a[1]*x**3 + a[2]*x**2 + a[3]*x + a[4]
        return value

    def poly4th_diff(self, a, x):
        """
        Function calculate the diff of 4th poly function
        :param a:
        :param x:
        :return:
        """
        value = 4*a[0]*x**3 + 3*a[1]*x**2 + 2*a[2]*x + a[3]
        return value

    def gait_generation(self, z_com, preview_time):
        """
        Fucntion generation walking gait
        :param z_com:
        :param preview_time:
        :return: zmp_ref, zmp_time, rankle_pos, rankle_vel, lankle_pos, lankle_vel, com_pos, com_vel, zmp_out
        """
        rankle_pos, rankle_vel, rankle_time, lankle_pos, lankle_vel, lankle_time = \
        self.cal_stance_foot([0.0, self.HipWidth / 2.0], 'Left')

        zmp_ref, zmp_time = self.cal_zmp_ref([0.0, 0.0], 'Left')
        # Thoi gian tao quy dao ZMP, LeftFoot, RightFoot la khac nhau  vi vay can
        # phai dong bo hoat thoi gian cua 3 quy dao
        time_end = num.array([rankle_time[-1], lankle_time[-1], zmp_time[-1]])
        longest_time = time_end.max()

        # Right ankle
        tmp_time = num.arange(rankle_time[-1], longest_time, self.dt)
        tmp_pos = num.matlib.repmat(rankle_pos[-1, :], len(tmp_time), 1)
        tmp_vel = num.matlib.repmat(rankle_vel[-1, :], len(tmp_time), 1)
        rankle_pos = num.vstack((rankle_pos, tmp_pos))
        rankle_vel = num.vstack((rankle_vel, tmp_vel))
        rankle_time = num.append(rankle_time, tmp_time)

        # Left ankle
        tmp_time = num.arange(lankle_time[-1], longest_time, self.dt)
        tmp_pos = num.matlib.repmat(lankle_pos[-1, :], len(tmp_time), 1)
        tmp_vel = num.matlib.repmat(lankle_vel[-1, :], len(tmp_time), 1)
        lankle_pos = num.vstack((lankle_pos, tmp_pos))
        lankle_vel = num.vstack((lankle_vel, tmp_vel))
        lankle_time = num.append(lankle_time, tmp_time)
        # zmp
        tmp_time = num.arange(zmp_time[-1], longest_time, self.dt)
        tmp_zmp = num.matlib.repmat(zmp_ref[-1, :], len(tmp_time), 1)
        zmp_ref = num.vstack((zmp_ref, tmp_zmp))
        zmp_time = num.append(zmp_time, tmp_time)

        # Them thoi gian chuan bi buoc di = Tcycle/2
        time_prepare = num.arange(0.0, self.Tcycle / 2 - self.dt, self.dt)
        len_time_prepare = len(time_prepare)
        # Left ankle
        tmp_pos = num.matlib.repmat(lankle_pos[0, :], len_time_prepare, 1)
        tmp_vel = num.matlib.repmat(lankle_vel[0, :], len_time_prepare, 1)
        lankle_pos = num.vstack((tmp_pos, lankle_pos))
        lankle_vel = num.vstack((tmp_vel, lankle_vel))
        lankle_time = num.append(time_prepare, self.Tcycle / 2 + lankle_time)
        # Right ankle
        tmp_pos = num.matlib.repmat(rankle_pos[0, :], len_time_prepare, 1)
        tmp_vel = num.matlib.repmat(rankle_vel[0, :], len_time_prepare, 1)
        rankle_pos = num.vstack((tmp_pos, rankle_pos))
        rankle_vel = num.vstack((tmp_vel, rankle_vel))
        rankle_time = num.append(time_prepare, self.Tcycle / 2 + rankle_time)
        # zmp ref
        tmp_zmp = num.matlib.repmat(zmp_ref[0, :], len_time_prepare, 1)
        zmp_ref = num.vstack((tmp_zmp, zmp_ref))
        zmp_time = num.append(time_prepare, self.Tcycle / 2 + zmp_time)
        ####################################################################
        # OK!
        # now, remove the duplicate data in time series
        ####################################################################
        # Left ankle: find and remove the duplicate time element
        duplicate_idx = None
        for index in range(len(lankle_time) - 1):
            if lankle_time[index] == lankle_time[index + 1]:
                if duplicate_idx is None:
                    duplicate_idx = index
                else:
                    duplicate_idx = num.append(duplicate_idx, index)
        lankle_pos = num.delete(lankle_pos, duplicate_idx, axis=0)
        lankle_vel = num.delete(lankle_vel, duplicate_idx, axis=0)
        lankle_time = num.delete(lankle_time, duplicate_idx)
        # Right ankle: find and remove the duplicate time element
        duplicate_idx = None
        for index in range(len(rankle_time) - 1):
            if rankle_time[index] == rankle_time[index + 1]:
                if duplicate_idx is None:
                    duplicate_idx = index
                else:
                    duplicate_idx = num.append(duplicate_idx, index)
        rankle_pos = num.delete(rankle_pos, duplicate_idx, axis=0)
        rankle_vel = num.delete(rankle_vel, duplicate_idx, axis=0)
        rankle_time = num.delete(rankle_time, duplicate_idx)
        # zmp reference: find and remove the duplicate time element
        duplicate_idx = None
        for index in range(len(zmp_time) - 1):
            if zmp_time[index] == zmp_time[index + 1]:
                if duplicate_idx is None:
                    duplicate_idx = index
                else:
                    duplicate_idx = num.append(duplicate_idx, index)
        zmp_ref = num.delete(zmp_ref, duplicate_idx, axis=0)
        zmp_time = num.delete(zmp_time, duplicate_idx)
        # Using cart-table model to generate CoM and ZPM
        com_pos, com_vel, zmp_out = self.cart_table(zmp_ref, z_com, zmp_time[-1], preview_time)
        return zmp_ref, zmp_time, rankle_pos, rankle_vel, lankle_pos, lankle_vel, com_pos, com_vel, zmp_out

    def cart_table(self, zmp_ref, z_com, cal_time, preview_time):
        """
        Function calculate CoM using cart-table model
        :param zmp_ref:
        :param z_com:
        :param cal_time:
        :param preview_time:
        :return:
        """
        dt = self.dt
        g = -9.810
        pre_time = num.arange(0.0, preview_time + self.dt, self.dt)
        pre_len = len(pre_time)
        insert_x = num.matlib.repmat(zmp_ref[-1, 0], pre_len, 1)
        insert_y = num.matlib.repmat(zmp_ref[-1, 1], pre_len, 1)

        foot_x = num.append(zmp_ref[:, 0], insert_x)
        foot_y = num.append(zmp_ref[:, 1], insert_y)

        mat_a = num.array([[1, dt, dt**2],
                           [0, 1, dt],
                           [0, 0, 1]])
        mat_b = num.array([[dt**3/6.0],
                           [dt**2/2.0],
                           [dt]])
        mat_c = num.array([1, 0, z_com/g])
        mat_d = num.array([0])

        # Error system
        zero = num.zeros((3, 1))
        phi_r0 = num.append(1.0, num.dot(-mat_c, mat_a))
        phi_r1 = num.hstack((zero, mat_a))
        mat_phi = num.vstack((phi_r0, phi_r1))
        mat_g = num.vstack((num.dot(-mat_c, mat_b), mat_b))
        mat_gr = num.vstack(([1, zero]))

        Q = num.zeros((4, 4))
        Q[0, 0] = 10.0**8
        H = 1.0
        # Riccati equation
        """Solve the discrete time lqr controller.
            x[k+1] = A x[k] + B u[k]
            cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
            """
        # first, try to solve the ricatti equation
        P = scipy.linalg.solve_discrete_are(mat_phi, mat_g, Q, H)
        # K=-(H+mat_g'*P*mat_g)^(-1)*mat_g'*P*mat_phi;
        K = -num.dot(num.dot(num.dot(1.0/(H + num.dot(num.dot(mat_g.T, P), mat_g)), mat_g.T), P), mat_phi)
        # xi = (eye(4,4)-mat_g*(H+mat_g'*P*mat_g)^(-1)*mat_g'*P)*mat_phi;
        xi = num.dot((num.eye(4) - num.dot(num.dot(mat_g, 1.0/(H + num.dot(num.dot(mat_g.T, P), mat_g))*mat_g.T), P)), mat_phi)

        # Now, solving the preview control problem
        x = num.array([[0], [0], [0]])
        y = num.array([[0], [0], [0]])
        xp = x
        yp = y

        ux = 0
        uy = 0
        time = num.arange(0.0, cal_time, self.dt, dtype=float)
        len_time = len(time)
        com_pos = None
        com_vel = None
        zmp_out = None
        # Loop start
        for index in range(len_time):
            p_x = num.dot(mat_c, x)  # Output zmp in x direction
            p_y = num.dot(mat_c, y)  # Output zmp in y direction
            e_x = foot_x[index] - p_x  # Error between the target ZMP(x)
            e_y = foot_y[index] - p_y  # Error between the target ZMP(y)

            X = num.vstack((e_x, x - xp))
            Y = num.vstack((e_y, y - yp))
            xp = x
            yp = y

            # x direction
            du_x = num.dot(K, X)
            t_x = num.arange(time[index], time[index] + preview_time, self.dt, dtype=float)

            for idx in range(1, len(t_x)):
                if foot_x[index + idx] - foot_x[index + idx - 1] != 0.0:
                    # gain_idx =  -(H+G'*P*G)^(-1)*G'*(xi')^(j-1)*P*GR;
                    gain_idx = -num.dot(num.dot(num.dot((1.0/(H + num.dot(num.dot(mat_g.T, P), mat_g)))*mat_g.T, num.linalg.matrix_power(xi.T, idx - 1)), P), mat_gr)
                    # gain_idx = gain_idx.ravel()
                    du_x = du_x + gain_idx*(foot_x[index + idx] - foot_x[index + idx - 1])
            ux = ux + du_x

            # y direction
            du_y = num.dot(K, Y)
            t_y = num.arange(time[index], time[index] + preview_time, self.dt, dtype=float)
            for idx in range(1, len(t_y)):
                if foot_y[index + idx] - foot_y[index + idx - 1] != 0.0:
                    # = -(H+G'*P*G)^(-1)*G'*(xi')^(j-1)*P*GR;
                    gain_idx = -num.dot(num.dot(num.dot((1.0 / (H + num.dot(num.dot(mat_g.T, P), mat_g))) * mat_g.T,
                                                        num.linalg.matrix_power(xi.T, idx - 1)), P), mat_gr)
                    du_y = du_y + gain_idx * (foot_y[index + idx] - foot_y[index + idx - 1])
            uy = uy + du_y

            x = num.dot(mat_a, x) + num.dot(mat_b, ux)
            y = num.dot(mat_a, y) + num.dot(mat_b, uy)

            if com_pos is None:
                com_pos = num.array([x[0], y[0], z_com])
                com_vel = num.array([x[1], y[1], 0.0])
                zmp_out = num.append(p_x, p_y)
            else:
                com_pos = num.vstack((com_pos, [x[0], y[0], z_com]))
                com_vel = num.vstack((com_vel, [x[1], y[1], 0]))
                zmp_out = num.vstack((zmp_out, num.append(p_x, p_y)))

        return com_pos, com_vel, zmp_out












