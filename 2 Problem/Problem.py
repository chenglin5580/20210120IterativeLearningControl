


################################ Package Import ####################################
import numpy as np
import matplotlib.pyplot as plt


################################ Problem Definition ####################################


class Virtual_problem(object):

    def __init__(self):

        self.x_dim = 2
        self.a_dim = 1

        self.x_0 = np.zeros(self.x_dim)
        self.t_0 = 0.0
        self.t_delta = 0.001

        self.flag_random_state_initial = False

    def fun_reset(self):

        if not self.flag_random_state_initial:
            self.x_0 = np.zeros(self.x_dim)
            self.x = np.copy(self.x_0)
            self.t_0 = 0.0
            self.t = np.copy(self.t_0)

            return self.x, self.t
        else:
            pass


    def fun_dynamics(self, t, x, u):

        x_dot0 = x[0]
        x_dot1 = x[1]
        x_dot2 = 3 * x_dot0 + u + 0.1 * np.sin(x_dot0)

        a = 1

        x_dot = np.array([x_dot1, x_dot2])
        return x_dot



    def fun_state_update(self, u):

        x_now = np.copy(self.x)
        k1 = self.fun_dynamics(0, x_now, u)
        k2 = self.fun_dynamics(0, x_now + self.t_delta * k1 / 2, u)
        k3 = self.fun_dynamics(0, x_now + self.t_delta * k2 / 2, u)
        k4 = self.fun_dynamics(0, x_now + self.t_delta * k3, u)
        x_next = x_now + self.t_delta * (k1 + 2 * k2 + 2 * k3 + k4) / 6

        self.x = x_next
        self.t += self.t_delta

        return self.x, self.t



class PID_controller(object):

    def __init__(self):

        self.pro = Virtual_problem()
        self.xd = np.array([1, 0])
        self.e_sum = 0

    def fun_controller(self, x, xd):

        kp = 10
        kd = 10
        ki = 10

        e_dot0 = xd[0] - x[0]
        e_dot1 = xd[1] - x[1]
        self.e_sum += e_dot0 * self.pro.t_delta

        u = kp * e_dot0 + ki *  self.e_sum + kd * e_dot1

        return u


    def fun_flight(self):

        x, t = self.pro.fun_reset()
        x_tra = np.empty([2, 0])
        xd_tra = np.empty([2, 0])
        u_tra = np.empty([1, 0])
        t_tra = np.empty([1, 0])


        for i in range(5000):

            u = self.fun_controller(x, self.xd)
            x, t = self.pro.fun_state_update(u)

            x_tra = np.hstack((x_tra, x.reshape([2, 1])))
            xd_tra = np.hstack((xd_tra, self.xd.reshape([2, 1])))
            u_tra = np.hstack((u_tra, u.reshape([1, 1])))
            t_tra = np.hstack((t_tra, t.reshape([1, 1])))



        plt.figure(1)
        plt.plot(t_tra[0, :], xd_tra[0, :], 'r-')
        plt.plot(t_tra[0, :], x_tra[0, :])

        plt.figure(2)
        plt.plot(t_tra[0, :], x_tra[0, :], 'r-')
        plt.plot(t_tra[0, :], xd_tra[0, :])

        plt.figure(3)
        plt.plot(t_tra[0, :], u_tra[0, :])

        plt.show()




if __name__ == '__main__':

    controller = PID_controller()
    controller.fun_flight()































































