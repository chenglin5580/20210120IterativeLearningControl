


################################ Package Import ####################################
import numpy as np
import matplotlib.pyplot as plt
from Problem import Virtual_problem as Virtual_problem


################################ Controller ####################################

class PID_controller(object):

    def __init__(self):

        self.pro = Virtual_problem()
        self.xd = np.array([1, 0])
        self.e_sum = 0

    def fun_controller(self, x, xd):

        kp = 8
        kd = 4
        ki = 4

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


        for i in range(15000):

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































































