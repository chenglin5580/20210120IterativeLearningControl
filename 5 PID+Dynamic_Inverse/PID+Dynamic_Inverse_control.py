


################################ Package Import ####################################
import numpy as np
import matplotlib.pyplot as plt
from Problem import Virtual_problem as Virtual_problem


################################ Controller ####################################

class PID_Dynamic_Inverse_controller(object):

    def __init__(self):

        self.pro = Virtual_problem()
        self.xd = np.array([1, 0])

        self.e_sum = 0.

    def fun_controller(self, x, xd):

        x_dot0 = x[0]
        x_dot1 = x[1]
        e_dot0 = xd[0] - x[0]
        e_dot1 = xd[1] - x[1]

        lambda_x = 4
        x_dot2 = 0 + 2 * lambda_x * e_dot1 + lambda_x **2 * e_dot0

        u1 = x_dot2 - 3 * x_dot0 - 0.1 * np.sin(x_dot0) + 4

        kp = lambda_x ** 2
        kd = 2 * lambda_x

        e_dot0 = xd[0] - x[0]
        e_dot1 = xd[1] - x[1]

        u2 = kp * e_dot0  + kd * e_dot1



        ki = (lambda_x / 2) ** 2 * 1
        s = lambda_x * e_dot0 + e_dot1
        self.e_sum += s * self.pro.t_delta
        u3 =  ki * self.e_sum

        omega = 0.99
        u = (1 - omega) * u1 + omega * u2  + 1. * u3

        return u


    def fun_flight(self):

        x, t = self.pro.fun_reset()
        x_tra = np.empty([2, 0])

        xd_tra = np.empty([2, 0])
        u_tra = np.empty([1, 0])
        t_tra = np.empty([1, 0])


        for i in range(5000):

            xd = np.array([i*self.pro.t_delta/5 + 1, 0.2])
            # xd = self.xd

            u = self.fun_controller(x, xd)
            x, t = self.pro.fun_state_update(u)

            x_tra = np.hstack((x_tra, x.reshape([2, 1])))
            xd_tra = np.hstack((xd_tra, xd.reshape([2, 1])))
            u_tra = np.hstack((u_tra, u.reshape([1, 1])))
            t_tra = np.hstack((t_tra, t.reshape([1, 1])))



        plt.figure(1)
        plt.plot(t_tra[0, :], xd_tra[0, :], 'r-')
        plt.plot(t_tra[0, :], x_tra[0, :])

        plt.figure(2)
        plt.plot(t_tra[0, :], xd_tra[1, :], 'r-')
        plt.plot(t_tra[0, :], x_tra[1, :])

        plt.figure(3)
        plt.plot(t_tra[0, :], u_tra[0, :])

        plt.show()






if __name__ == '__main__':

    controller = PID_Dynamic_Inverse_controller()
    controller.fun_flight()































































