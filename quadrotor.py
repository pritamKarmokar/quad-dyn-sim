
from collections import namedtuple
from math import sin, cos, tan, atan, atan2, pi, tau
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from config import *

class Quadrotor:
    def __init__(self, m=MASS, Ixx=I_XX, Iyy=I_YY, Izz=I_ZZ):
        # set inertia parameters
        self.set_inertia_params()

        # set acceleration due to gravity
        self.g = ACC_GRAVITY

        # set gains
        self.set_gains()

        # set initial state
        self.set_init_state()

        # set commanded accelerations
        self.ax = AX
        self.ay = AY
        self.az = AZ

        # compute commanded attitude using commanded accelerations
        self.phi_c = atan(self.ay * cos(self.theta)/(self.g - self.az + 1e-16))     # <- check for az sign
        self.theta_c = atan(self.ax / (self.az - self.g + 1e-16))                  # <- check for az sign
        self.psi_c = 0

        # compute required force
        self.F = (self.g - self.az) * (self.INERTIA.m / (1e-16 + cos(self.phi)*cos(self.theta)))

        # compute required torque
        self.tau_phi = self.GAINS.KP_phi*self.norm_ang(self.phi_c - self.phi) + self.GAINS.KD_phi*(0-self.p)
        self.tau_theta = self.GAINS.KP_theta*self.norm_ang(self.theta_c - self.theta) + self.GAINS.KD_theta*(0-self.q)
        self.tau_psi = self.GAINS.KP_psi*self.norm_ang(self.psi_c - self.psi) + self.GAINS.KD_psi*(0-self.r)



    @staticmethod
    def norm_ang(ang):
        return np.sign(ang) * (abs(ang)%tau)


    def compute_force_and_torque(self):
        # compute commanded attitude using commanded accelerations
        self.phi_c = atan(self.ay * cos(self.theta)/(self.g - self.az + 1e-16))     # <- check for az sign
        self.theta_c = atan(self.ax / (self.az - self.g + 1e-16))                  # <- check for az sign
        self.psi_c = 0

        # compute required force
        self.F = (self.g - self.az) * (self.INERTIA.m / (1e-16 + cos(self.phi)*cos(self.theta)))
        

        # compute required torque
        self.tau_phi = self.GAINS.KP_phi*self.norm_ang(self.phi_c - self.phi) + self.GAINS.KD_phi*(0-self.p)
        self.tau_theta = self.GAINS.KP_theta*self.norm_ang(self.theta_c - self.theta) + self.GAINS.KD_theta*(0-self.q)
        self.tau_psi = self.GAINS.KP_psi*self.norm_ang(self.psi_c - self.psi) + self.GAINS.KD_psi*(0-self.r)


    def set_inertia_params(self):
        """Set inertia params - mass and moments of inertia Ixx, Iyy and Izz (diagonal elements of inertia tensor) - for drone. 
        """
        InertiaParams = namedtuple('InertiaParams', 'm Ixx Iyy Izz')
        self.INERTIA = InertiaParams(MASS, I_XX, I_YY, I_ZZ)


    def set_gains(self):
        """set proportional and derivative gains for euler angles roll(φ), pitch(θ), yaw(ψ) and altitude
        """
        Gains = namedtuple('Gains', 'KP_phi KD_phi KP_theta KD_theta KP_psi KD_psi KP_z KD_z')
        self.GAINS = Gains(K_P_PHI, K_D_PHI, K_P_THETA, K_D_THETA, K_P_PSI, K_D_PSI, K_P_Z, K_D_Z)


    def set_init_state(self):
        print('\n\n INITIALIZING QUADROTOR STATES')
        print('------------------------------\n')
        self.pN = PN
        self.pE = PE
        self.pH = PH
        self.U = U
        self.V = V
        self.W = W
        self.phi = PHI
        self.theta = THETA
        self.psi = PSI
        self.p = P
        self.q = Q
        self.r = R

        self.state = np.array([self.pN,
                               self.pE,
                               self.pH,
                               self.U,
                               self.V,
                               self.W,
                               self.phi,
                               self.theta,
                               self.psi,
                               self.p,
                               self.q,
                               self.r])


    def print_states(self):
        """helper function to print states"""
        print(f'pos=({self.pN:.2f}, ' + 
              f'{self.pE:.2f}, ' + 
              f'{self.pH:.2f})  vel=(' +
              f'{self.U:.2f}, ' +
              f'{self.V:.2f}, ' +
              f'{self.W:.2f})  att=(' +
              f'{self.phi:.2f}, ' +
              f'{self.theta:.2f}, ' +
              f'{self.psi:.2f})  att_rate=(' +
              f'{self.p:.2f}, ' +
              f'{self.q:.2f}, ' +
              f'{self.r:.2f})  FaT=(' +
              f'{self.F:.2f}, ' +
              f'{self.tau_phi:.2f}, ' +
              f'{self.tau_theta:.2f}, ' + 
              f'{self.tau_psi:.2f})'
            )    

    def step(self, delta_t=DELTA_T, integrator=EULER):
        state = self.get_state()

        updated_state = self.update_state(state, delta_t, integrator)
        
        # self.print_states()
        return updated_state


    def get_state(self):
        self.state = np.array([self.pN,
                               self.pE,
                               self.pH,
                               self.U,
                               self.V,
                               self.W,
                               self.phi,
                               self.theta,
                               self.psi,
                               self.p,
                               self.q,
                               self.r])

        return self.state


    def update_state(self, state, delta_t, integrator=EULER):
        self.compute_force_and_torque()

        if integrator==EULER:
            self.state = self.euler_update_state(state, delta_t)
        elif integrator==RK45:
            self.state = self.rk45_update_state(state, delta_t)

        self.pN = self.state[0]
        self.pE = self.state[1]
        self.pH = self.state[2]
        self.U = self.state[3]
        self.V = self.state[4]
        self.W = self.state[5]
        self.phi = self.state[6]
        self.theta = self.state[7]
        self.psi = self.state[8]
        self.p = self.state[9]
        self.q = self.state[10]
        self.r = self.state[11]

        return self.state


    def euler_update_state(self, state, delta_t):
        num_inner_loop = NUM_INNER_LOOP
        inner_loop_step = delta_t / num_inner_loop

        for i in range(num_inner_loop):
            F_app = self.F
            state_dot = self.eval_state_dot(state, F_app)

            state += state_dot*inner_loop_step

        return state



    def rk45_update_state(self, state, delta_t):
        num_inner_loop = NUM_INNER_LOOP
        inner_loop_step = delta_t / num_inner_loop

        for i in range(num_inner_loop):
            F_app = self.F
            k1 = self.eval_state_dot(state, F_app)
            k2 = self.eval_state_dot(state + k1*(inner_loop_step/2), F_app)
            k3 = self.eval_state_dot(state + k2*(inner_loop_step/2), F_app)
            k4 = self.eval_state_dot(state + k3*inner_loop_step, F_app)

            state += (k1 + 2*k2 + 2*k3 + k4) * (inner_loop_step/6)

        return state


    def eval_state_dot(self, state, F):
        state_dot = np.zeros_like(state)
        pN = state[0]
        pE = state[1]
        pH = state[2]
        u = state[3]
        v = state[4]
        w = state[5]
        phi = state[6]
        theta = state[7]
        psi = state[8]
        p = state[9]
        q = state[10]
        r = state[11]

        # pN_dot, pE_dot, pH_dot
        # -----------------------
        # construct rotation matrix transforming body frame (A) to local NED (N)
        self.A_R_N = np.array([[cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)],
                               [cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)],
                               [-sin(theta),         sin(phi)*cos(theta),                            cos(phi)*cos(theta)]])
        
        # transform and compute velocities of inertial frame quantities pN, pE and h
        vel_NEU = self.A_R_N @ np.array([[u],
                                         [v],
                                         [w]])

        state_dot[:3] = vel_NEU.flatten()

        # u_dot, v_dot, w_dot
        # -------------------
        # computer velocity rates
        vel_dot = np.array([[r*v - q*w],
                            [p*w - r*u],
                            [q*u - p*v]]) \
                + np.array([[-self.g*sin(theta)],
                            [self.g*cos(theta)*sin(phi)],
                            [self.g*cos(theta)*cos(phi)]]) \
                + np.array([[0],
                            [0],
                            [-F/self.INERTIA.m]])

        state_dot[3:6] = vel_dot.flatten()

        # phi_dot, theta_dot, psi_dot
        # ---------------------------
        # compute the transform matrix
        transform_mat = np.array([[1, sin(phi)*tan(theta), cos(phi)*tan(theta)],
                                  [0, cos(phi),            -sin(phi)],
                                  [0, sin(phi)/(cos(theta)+1e-16), cos(phi)/(cos(theta)+1e-16)]])

        # compute attitude rate dynamics
        attitude_dot = transform_mat @ np.array([[p],
                                                 [q],
                                                 [r]])

        state_dot[6:9] = attitude_dot.flatten()

        # p_dot, q_dot, r_dot
        # -------------------
        # compute coriolis term
        coriolis = np.array([[((self.INERTIA.Iyy - self.INERTIA.Izz)/self.INERTIA.Ixx)*q*r],
                             [((self.INERTIA.Izz - self.INERTIA.Ixx)/self.INERTIA.Iyy)*p*r],
                             [((self.INERTIA.Ixx - self.INERTIA.Iyy)/self.INERTIA.Izz)*p*q]])

        # compute F/m analog in Euler equations - τ/I
        angular_accel = np.array([[(1/self.INERTIA.Ixx)*self.tau_phi],
                                  [(1/self.INERTIA.Iyy)*self.tau_theta],
                                  [(1/self.INERTIA.Izz)*self.tau_psi]])

        # compute angular velocity rate dynamics
        ang_vel_dot = coriolis + angular_accel

        state_dot[9:] = ang_vel_dot.flatten()

        return state_dot

        
    def simulate(self, delta_t=DELTA_T, final_time=30, integrator=EULER):
        time = np.array([i for i in np.arange(0,final_time,DELTA_T)])

        states = np.expand_dims(self.get_state(),0)
        forces = np.array([[self.F, self.tau_phi, self.tau_theta, self.tau_psi]])
        for i in time:
            state = self.step(delta_t, integrator)
            states = np.concatenate((states, np.expand_dims(state,0)), axis=0)
            forces = np.concatenate((forces,np.array([[self.F, self.tau_phi, self.tau_theta, self.tau_psi]])),axis=0)

        return time, states, forces


def add_plot_states(axs, t, quad_states):
    axs[0,0].plot(t,quad_states[1:,0],  alpha=0.7)
    axs[1,0].plot(t,quad_states[1:,1],  alpha=0.7)
    axs[2,0].plot(t,quad_states[1:,2],  alpha=0.7)
    axs[0,1].plot(t,quad_states[1:,3],  alpha=0.7)
    axs[1,1].plot(t,quad_states[1:,4],  alpha=0.7)
    axs[2,1].plot(t,quad_states[1:,5],  alpha=0.7)
    axs[0,2].plot(t,quad_states[1:,6],  alpha=0.7)
    axs[1,2].plot(t,quad_states[1:,7],  alpha=0.7)
    axs[2,2].plot(t,quad_states[1:,8],  alpha=0.7)
    axs[0,3].plot(t,quad_states[1:,9],  alpha=0.7)
    axs[1,3].plot(t,quad_states[1:,10], alpha=0.7)
    axs[2,3].plot(t,quad_states[1:,11], alpha=0.7)
    axs[0,0].plot(t,quad_states[1:,0],  alpha=0.7)
    axs[0,0].fill_between(t,quad_states[1:,0],  0, alpha=0.1)
    axs[1,0].fill_between(t,quad_states[1:,1],  0, alpha=0.1)
    axs[2,0].fill_between(t,quad_states[1:,2],  0, alpha=0.1)
    axs[0,1].fill_between(t,quad_states[1:,3],  0, alpha=0.1)
    axs[1,1].fill_between(t,quad_states[1:,4],  0, alpha=0.1)
    axs[2,1].fill_between(t,quad_states[1:,5],  0, alpha=0.1)
    axs[0,2].fill_between(t,quad_states[1:,6],  0, alpha=0.1)
    axs[1,2].fill_between(t,quad_states[1:,7],  0, alpha=0.1)
    axs[2,2].fill_between(t,quad_states[1:,8],  0, alpha=0.1)
    axs[0,3].fill_between(t,quad_states[1:,9],  0, alpha=0.1)
    axs[1,3].fill_between(t,quad_states[1:,10], 0, alpha=0.1)
    axs[2,3].fill_between(t,quad_states[1:,11], 0, alpha=0.1)
    

def add_plot_wrench(axs, t, forces):
    axs[0].plot(t, forces[1:,0],alpha=0.7)
    axs[1].plot(t, forces[1:,1],alpha=0.7)
    axs[2].plot(t, forces[1:,2],alpha=0.7)
    axs[3].plot(t, forces[1:,3],alpha=0.7)
    axs[0].fill_between(t, forces[1:,0],0,alpha=0.1)
    axs[1].fill_between(t, forces[1:,1],0,alpha=0.1)
    axs[2].fill_between(t, forces[1:,2],0,alpha=0.1)
    axs[3].fill_between(t, forces[1:,3],0,alpha=0.1)


def add_plot_traj(axs, states):
    axs.plot3D(states[1:,0], states[1:,1], states[1:,2], alpha=0.8)
    axs.scatter3D(states[1:,0], states[1:,1], states[1:,2], c=states[1:,2], cmap='plasma',s=3,alpha=0.7)

def set_state_plot_titles(axs):
    axs[0,0].set_title(r'$\mathbf{p_n}$')
    axs[1,0].set_title(r'$\mathbf{p_e}$')
    axs[2,0].set_title(r'$\mathbf{p_h}$')
    axs[0,1].set_title(r'$\mathbf{u}$')
    axs[1,1].set_title(r'$\mathbf{v}$')
    axs[2,1].set_title(r'$\mathbf{w}$')
    axs[0,2].set_title(r'$\mathbf{\phi}$')
    axs[1,2].set_title(r'$\mathbf{\theta}$')
    axs[2,2].set_title(r'$\mathbf{\psi}$')
    axs[0,3].set_title(r'$\mathbf{p}$')
    axs[1,3].set_title(r'$\mathbf{q}$')
    axs[2,3].set_title(r'$\mathbf{r}$')

def set_wrench_plot_titles(axs):
    axs[0].set_title(r'$\mathbf{F}$')
    axs[1].set_title(r'$\mathbf{\tau_{\phi}}$')
    axs[2].set_title(r'$\mathbf{\tau_{\theta}}$')
    axs[3].set_title(r'$\mathbf{\tau_{\psi}}$')

def main():
    quadrotor = Quadrotor()
    t, quad_states, forces = quadrotor.simulate(delta_t=DELTA_T, final_time=FINAL_TIME, integrator=EULER)

    quadrotor.set_init_state()
    t2, quad_states2, forces2 = quadrotor.simulate(delta_t=DELTA_T, final_time=FINAL_TIME, integrator=RK45)
    # quadrotor.set_init_state()
    # t3, quad_states3, forces3 = quadrotor.simulate(delta_t=DELTA_T*4, final_time=FINAL_TIME, integrator=EULER)

    plt.style.use('seaborn-whitegrid')

    DPI = 100
    # --------------------------------
    f1, axs = plt.subplots(3, 4, sharex=True, figsize=(8,6), dpi=DPI,gridspec_kw={'hspace': 0.2, 'wspace':0.4})
    f1.suptitle(r'$\mathbf{Quadrotor\ states}$')
    fig_manager = plt.get_current_fig_manager()
    fig_manager.window.wm_geometry("+0+0")

    set_state_plot_titles(axs)

    add_plot_states(axs, t, quad_states)
    add_plot_states(axs, t2, quad_states2)
    # add_plot_states(axs, t3, quad_states3)    

    f1.show()

    # --------------------------------
    f2, axs = plt.subplots(1, 4, sharex=True, figsize=(8,2), dpi=DPI,gridspec_kw={'hspace': 0.2, 'wspace':0.4})
    f2.suptitle(r'$\mathbf{Wrench}$')
    fig_manager = plt.get_current_fig_manager()
    fig_manager.window.wm_geometry("+801+0")

    set_wrench_plot_titles(axs)

    add_plot_wrench(axs, t, forces)
    add_plot_wrench(axs, t2, forces2)
    # add_plot_wrench(axs, t3, forces3)

    f2.show()

    #-----------------------------------
    f3 = plt.figure(figsize=(8,3.35), dpi=DPI)
    axs = f3.add_subplot(111,projection='3d')
    f3.suptitle(r'$\mathbf{Quadrotor\ trajectory}$')
    axs.set(xlabel=r'$x\ (m)$', ylabel=r'$y\ (m)$', zlabel=r'$z\ (m)$')
    fig_manager = plt.get_current_fig_manager()
    fig_manager.window.wm_geometry("+801+265")

    add_plot_traj(axs, quad_states)
    add_plot_traj(axs, quad_states2)

    f3.show()


    # plt.draw()
    # plt.waitforbuttonpress(0)
    # plt.close('all')

    plt.show()


if __name__=='__main__':
    main()


