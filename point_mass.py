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
from Vizualize import QuadPlot

class Quadcopter:

    def __init__(self, x_init=None):

        # Dynamics params
        self.mass = 2.856

        self.max_F = 59.844

        # Definite intial conditions
        if x_init == None:
            self.x = np.zeros((12,1))


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

        self.desired_forces_.Fz = throttle*self.max_F
        # self.desired_forces_.Fz = throttle

        # Calc acutal output with low-pass filters

        # tauF = self.tau_up_F if (self.desired_forces_.Fz > self.applied_forces_.Fz) else self.tau_down_F



        # Apply other forces (i.e. wind)

        self.actual_forces_.Fz = - self.desired_forces_.Fz


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
        # att = np.array([phi, theta, psi])
        # ang_vel = np.array([p, q, r])
        force = np.array([self.actual_forces_.Fx, self.actual_forces_.Fy, self.actual_forces_.Fz])
        # torque = np.array([self.actual_forces_.l/self.Jx, self.actual_forces_.m/self.Jy, self.actual_forces_.n/self.Jz])
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
        # R_body_veh = np.array([[ct*cpsi, ct*spsi, -st],
        #                 [sp*st*cpsi-cp*spsi, sp*st*spsi+cp*cpsi, sp*ct],
        #                 [cp*st*cpsi+sp*spsi, cp*st*spsi-sp*cpsi, cp*ct]])

        grav = 9.81
        fg_body = np.array([-self.mass*grav*st, self.mass*grav*ct*sp, self.mass*grav*ct*cp])
        # gravity = np.array([0, 0, -9.81])
        # f_grav = np.matmul(rot_posdot.T, gravity)
        fg_body = np.reshape(fg_body, (3,1))

        rot_posdot = np.array([[ct*cpsi, sp*st*cpsi-cp*spsi, cp*st*cpsi+sp*spsi],
                        [ct*spsi, sp*st*spsi+cp*cpsi, cp*st*spsi-sp*cpsi],
                        [st, -sp*ct, -cp*ct]])

        pos_dot = np.matmul(rot_posdot, vel)
        # vel_dot = (1./self.mass)*force + (1./self.mass)*fg_body
        vel_dot = np.array([0., 0., (self.actual_forces_.Fz/self.mass)+(grav)])
        vel_dot = np.reshape(vel_dot, (3,1))
        # rot_attdot = np.array([[1., sp*tt, cp*tt], [0., cp, -sp], [0., (sp/ct), (cp/ct)]])
        # att_dot = np.matmul(rot_attdot, ang_vel)
        # ang_veldot = np.array([((self.Jy-self.Jz)/self.Jx)*q*r, ((self.Jz-self.Jx)/self.Jy)*p*r, ((self.Jx-self.Jy)/self.Jz)*p*q]) + torque

        # xdot
        xdot = np.zeros((12,1))
        # xdot[0:3] = pos_dot
        # xdot[2] = -xdot[2] # convert from hdot to pddot
        xdot[2] = w # convert from hdot to pddot
        xdot[3:6] = vel_dot
        # print "force", force
        # print "f/m", force/self.mass
        # print "wdot", xdot[5]
        # xdot[6:9] = att_dot
        # xdot[9:] = ang_veldot

        # apply propagation
        # print "XDOT"
        # print xdot
        self.x = self.x + xdot*dt

        # print "force", force

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
    plotter = QuadPlot()

    # Lets Fly :)
    dt = 0.01
    t = 0.0

    # Altitude Hold
    throttle_eq = quad.mass*9.81/quad.max_F
    alt_controller_ = PID(p=0.01, i=0.00, d=-0.1)
    alt_c = 10.0

    for i in range(10000):
        t += dt
        phi_c = 0.0
        theta_c = 0.0
        psirate_c = 0.0
        # throttle_c = quad.mass*9.81/quad.max_F + 0.01
        throttle_c = alt_controller_.computePID(alt_c, -quad.x[2], dt) + throttle_eq # eq force = -28.01736
        # error = alt_c + quad.x[2]
        # kp = 0.001
        # throttle_c = kp*error + throttle_eq
        quad.force_and_moments(phi_c, theta_c, psirate_c, throttle_c, dt)
        # print quad.x
        if (i%10 == 0):
            plotter.plot(quad.x)
            print "--------------------"
            print "iteration #", i
            print "pos:", quad.x[0], quad.x[1], quad.x[2], quad.x[5]
            print "rot:", quad.x[6], quad.x[7], quad.x[8]
            print "throt_c", throttle_c - throttle_eq
            print "error", alt_controller_.last_error_
            print "time: ", t
            # time.sleep(0.1)
