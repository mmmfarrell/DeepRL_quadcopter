'''
    This file defines a quadcopter class that simulates quadcopter dynamics.

    # State vector x = (12,1)
    pn, pe, pd, u, v, w, phi, theta, psi, p, q, r

    # Actions u = (4,1)

'''
import numpy as np
from math import *
from simple_pid import PID
import time

class Quadcopter:

    def __init__(self, x_init=None):

        # Dynamics params
        self.mass = 2.856
        self.linear_mu = 0.2
        self.angular_mu = 0.3
        self.ground_effect = [-55.3516, 181.8265, -203.9874, 85.3735, -7.6619]

        self.max_l = 6.5080
        self.max_m = 5.087
        self.max_n = 0.25
        self.max_F = 59.844

        self.Jx = 0.07
        self.Jy = 0.08
        self.Jz = 0.12

        self.J = np.diag([self.Jx, self.Jy, self.Jz])

        # Filter Outputs
        self.tau_up_l = 0.1904
        self.tau_up_m = 0.1904
        self.tau_up_n = 0.1644
        self.tau_up_F = 0.1644
        self.tau_down_l = 0.1904
        self.tau_down_m = 0.1904
        self.tau_down_n = 0.2164
        self.tau_down_F = 0.2164

        # Wind
        self.wind_n = 0.0
        self.wind_e = 0.0
        self.wind_d = 0.0

        # Definite intial conditions
        if x_init == None:
            self.x = np.zeros((12,1))

        # Init controllers
        self.roll_controller_ = PID(p=0.1, i=0.0, d=0.0)
        self.pitch_controller_ = PID(p=25.0, i=0.0, d=8.0)
        self.yaw_controller_ = PID(p=25.0, i=0.0, d=0.0)

        # Forces
        self.desired_forces_ = Forces()
        self.actual_forces_ = Forces()
        self.applied_forces_ = Forces()

    def force_and_moments(self, phi_c, theta_c, psi_rate_c, throttle, dt):

        # unpack state
        phi = self.x[6]
        theta = self.x[7]
        # psi = self.x[8]
        p = self.x[9]
        q = self.x[10]
        r = self.x[11]

        # Compute desired forces
        self.desired_forces_.l = self.roll_controller_.computePID(phi_c, phi, dt, p)
        self.desired_forces_.m = self.roll_controller_.computePID(theta_c, theta, dt, q)
        print "PID"
        print psi_rate_c
        print r
        self.desired_forces_.n = self.roll_controller_.computePID(psi_rate_c, r, dt)
        self.desired_forces_.Fz = throttle*self.max_F

        # Calc acutal output with low-pass filters
        taul = self.tau_up_l if (self.desired_forces_.l > self.applied_forces_.l) else self.tau_down_l
        taum = self.tau_up_m if (self.desired_forces_.m > self.applied_forces_.m) else self.tau_down_m
        taun = self.tau_up_n if (self.desired_forces_.n > self.applied_forces_.n) else self.tau_down_n
        tauF = self.tau_up_F if (self.desired_forces_.Fz > self.applied_forces_.Fz) else self.tau_down_F

        # Calc alpha for filter
        alpha1 = dt/(taul + dt)
        alpham = dt/(taum + dt)
        alphan = dt/(taun + dt)
        alphaF = dt/(tauF + dt)

        # Apply discrete first-order filter
        self.applied_forces_.l = self.sat(self.desired_forces_.l, self.max_l, -1.0*self.max_l)
        self.applied_forces_.m = self.sat(self.desired_forces_.m, self.max_m, -1.0*self.max_m)
        self.applied_forces_.n = self.sat(self.desired_forces_.n, self.max_n, -1.0*self.max_n)
        self.applied_forces_.Fz = self.sat(self.desired_forces_.Fz, self.max_F, 0.0)
        # self.applied_forces_.l = self.sat((1 - alpha1)*self.applied_forces_.l + alpha1*self.desired_forces_.l, self.max_l, -1.0*self.max_l)
        # self.applied_forces_.m = self.sat((1 - alpha1)*self.applied_forces_.m + alpha1*self.desired_forces_.m, self.max_m, -1.0*self.max_m)
        # self.applied_forces_.n = self.sat((1 - alpha1)*self.applied_forces_.n + alpha1*self.desired_forces_.n, self.max_n, -1.0*self.max_n)
        # self.applied_forces_.Fz = self.sat((1 - alpha1)*self.applied_forces_.Fz + alpha1*self.desired_forces_.Fz, self.max_F, 0.0)

        # TODO add ground effect
        ground_effect = 0.0

        # TODO add Wind effect
        ur = 0.0
        vr = 0.0
        wr = 0.0

        # Apply other forces (i.e. wind)
        self.actual_forces_.Fx = -1.0*self.linear_mu*ur
        self.actual_forces_.Fy = -1.0*self.linear_mu*vr
        self.actual_forces_.Fz = -1.0*self.linear_mu*wr - self.applied_forces_.Fz - ground_effect
        self.actual_forces_.l = -1.0*self.angular_mu*p + self.applied_forces_.l
        self.actual_forces_.m = -1.0*self.angular_mu*q + self.applied_forces_.m
        self.actual_forces_.n = -1.0*self.angular_mu*r + self.applied_forces_.n

        self.apply_forces(dt)

    def apply_forces(self, dt):

        # Unpack state
        pn = self.x[0]
        pe = self.x[1]
        pd = self.x[2]
        u = self.x[3]
        v = self.x[4]
        w = self.x[5]
        phi = self.x[6]
        theta = self.x[7]
        psi = self.x[8]
        p = self.x[9]
        q = self.x[10]
        r = self.x[11]

        # pos = np.array([[pn], [pe], [pd]])
        pos = np.array([pn, pe, pd])
        vel = np.array([u, v, w])
        att = np.array([phi, theta, psi])
        ang_vel = np.array([p, q, r])
        force = np.array([self.actual_forces_.Fx, self.actual_forces_.Fy, self.actual_forces_.Fz])
        torque = np.array([self.actual_forces_.l/self.Jx, self.actual_forces_.m/self.Jy, self.actual_forces_.n/self.Jz])
        force = np.reshape(force, (3,1))

        # position dot
        # Calc trigs
        cp = cos(phi)
        sp = sin(phi)
        ct = cos(theta)
        st = sin(theta)
        tt = tan(theta)
        cpsi = cos(psi)
        spsi = sin(psi)

        # calc rotation matrix
        rot_posdot = np.array([[ct*cpsi, sp*st*cpsi-cp*spsi, cp*st*cpsi+sp*spsi],
                        [ct*spsi, sp*st*spsi+cp*cpsi, cp*sp*spsi-sp*cpsi],
                        [st, -sp*ct, -cp*ct]])

        gravity = np.array([0, 0, -9.81])
        f_grav = np.matmul(rot_posdot.T, gravity)
        f_grav = np.reshape(f_grav, (3,1))

        pos_dot = np.matmul(rot_posdot, vel)
        vel_dot = np.array([r*v-q*w, p*w-r*u, q*u-p*v]) + (1./self.mass)*force + f_grav
        rot_attdot = np.array([[1., sp*tt, cp*tt], [0., cp, -sp], [0., (sp/ct), (cp/ct)]])
        att_dot = np.matmul(rot_attdot, ang_vel)
        ang_veldot = np.array([((self.Jy-self.Jz)/self.Jx)*q*r, ((self.Jz-self.Jx)/self.Jy)*p*r, ((self.Jx-self.Jy)/self.Jz)*p*q]) + torque

        # xdot
        xdot = np.zeros((12,1))
        xdot[0:3] = pos_dot
        xdot[3:6] = vel_dot
        xdot[6:9] = att_dot
        xdot[9:] = ang_veldot

        # apply propagation
        print "XDOT"
        print xdot
        self.x = self.x + xdot*dt

    def sat(self, x, _max, _min):
        if (x > _max):
            print "SAT MAX"
            return _max
        elif (x < _min):
            print "SAT MIN"
            return _min
        else:
            return x

class Forces:
    def __init__(self):
        self.l = 0.0
        self.m = 0.0
        self.n = 0.0
        self.Fx = 0.0
        self.Fy = 0.0
        self.Fz = 0.0


##############################
#### Main Function to Run ####
##############################
if __name__ == '__main__':

    # init path_manager_base object
    quad = Quadcopter()

    # Lets Fly :)

    for i in range(1000):
        print "iteration #", i
        phi_c = 0.2
        theta_c = 0.0
        psirate_c = 0.0
        throttle_c = quad.mass*9.81/quad.max_F + 0.1
        quad.force_and_moments(phi_c, theta_c, psirate_c, throttle_c, 0.001)
        print quad.x
        time.sleep(0.01)
