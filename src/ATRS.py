import os, sys
from time import perf_counter_ns
from queue import Full

import numpy as np
import casadi as cs

from mpc.mpc_generator import MpcModule
from utils.config import SolverParams

class ATRS(MpcModule):
    """Dummy class not intended to be used later on
    """
    def __init__(self, master_state, slave_state, solver_param:SolverParams, plot_config, plot_queues):
        super().__init__(solver_param)
        self.master_state = master_state
        self.slave_state = slave_state
        self.plot_config = plot_config
        self.plot_queues = plot_queues
        
        self.past_states = [[], []]
        self.past_u = [[], []]
        self.state_transitions_t = {'coupling':0, 'formation':None, 'decoupling':None}
        self.past_states[0].extend(self.master_state)
        self.past_states[1].extend(self.slave_state)
        self.times = {'update':[], 'plot':[]}
        self.past_formation_ang_deviation = [0]

    def update_pos(self, u, lines_x, lines_y, adjust_theta=False):
        """Updates the slave and master position

        Args:
            u (list of length 1): control signals for the master and slave
        """
        start_time = perf_counter_ns()

        # This is not written to handle more inputs than 1 step ahead
        assert(len(u) == 4)

        # Save the control signals
        self.past_u[0].extend(u[:2])
        self.past_u[1].extend(u[2:])
        state = self.dynamics_rk4_dual(self.master_state+self.slave_state, np.array(u))
        if adjust_theta:
            if state[2] < -2*np.pi: 
                state[2] += 2*np.pi
            elif state[2] > 2*np.pi: 
                state[2] -= 2*np.pi
            if state[5] < -2*np.pi: 
                state[5] += 2*np.pi
            elif state[5] > 2*np.pi: 
                state[5] -= 2*np.pi
        self.master_state = [float(state[0]), 
                             float(state[1]), 
                             float(state[2])]

        self.slave_state = [float(state[3]), 
                             float(state[4]), 
                             float(state[5])]
        
        # Save the new states
        self.past_states[0].extend(self.master_state)
        self.past_states[1].extend(self.slave_state)

        if not np.any(lines_x == None):
            # closest_line, _ = self.get_closest_line(cs.vertcat(self.master_state[0], self.master_state[1]), lines_x, lines_y)              
            s0 = cs.vertcat(lines_x[0], lines_y[0])
            s1 = cs.vertcat(lines_x[1], lines_y[1])
            s2 = cs.vertcat(lines_x[2], lines_y[2])

            formation_angle_ref = self.calc_formation_angle_ref(self.master_state, s0, s1, s2)
            formation_ang_deviation = self.calc_formation_angle_error(*(self.master_state[:2]), *(self.slave_state[:2]), formation_angle_ref) 
            self.past_formation_ang_deviation.append(float(formation_ang_deviation))

        self.times['update'].append(perf_counter_ns() - start_time)

    def get_states(self):
        """Returns the state of the master and slave

        Returns:
            list: [x_master, y_master, theta_master, x_slave, y_slave, theta_slave]
        """
        return self.master_state + self.slave_state

    def get_prev_states(self):
        x_master = self.past_states[0][0::3]
        y_master = self.past_states[0][1::3]
        theta_master = self.past_states[0][2::3]
        x_slave = self.past_states[1][0::3]
        y_slave = self.past_states[1][1::3]
        theta_slave = self.past_states[1][2::3]
        return x_master, y_master, theta_master, x_slave, y_slave, theta_slave
    
    def get_prev_control_signals(self):
        v_master = self.past_u[0][0::2]
        ang_master = self.past_u[0][1::2]
        v_slave = self.past_u[1][0::2]
        ang_slave = self.past_u[1][1::2]

        return v_master, ang_master, v_slave, ang_slave

    def get_prev_acc(self):
        if len(self.past_u[0]) < 4:
            return [0,0,0,0] 
        master_lin_acc = np.diff(self.past_u[0][-4::2]) / self.solver_param.base.ts
        master_ang_acc = np.diff(self.past_u[0][-3::2]) / self.solver_param.base.ts
        slave_lin_acc = np.diff(self.past_u[1][-4::2]) / self.solver_param.base.ts
        slave_ang_acc = np.diff(self.past_u[1][-3::2]) / self.solver_param.base.ts

        return [master_lin_acc, master_ang_acc, slave_lin_acc, slave_ang_acc]

    def get_prev_constraint_violation(self):
        x_master, y_master, theta_master, x_slave, y_slave, theta_slave = self.get_prev_states()
        return[ ((x_master[i]-x_slave[i])**2 + (y_master[i]-y_slave[i])**2)**0.5 - self.solver_param.base.constraint['d'] for i in range(len(x_master))]

    def calc_constraint_violation(self, state=None):
        """Calculates the distance between the ATRs, not the constraint violation.

        Args:
            state ([type], optional): [description]. Defaults to None.

        Raises:
            NotImplementedError: [description]

        Returns:
            [type]: [description]
        """
        if state == None:
            x_master = self.past_states[0][0::3]
            y_master = self.past_states[0][1::3]
            x_slave = self.past_states[1][0::3]
            y_slave = self.past_states[1][1::3]
        elif state == 'formation':
            x_master = self.past_states[0][0::3][self.state_transitions_t['formation']:self.state_transitions_t['decoupling']]
            y_master = self.past_states[0][1::3][self.state_transitions_t['formation']:self.state_transitions_t['decoupling']]
            x_slave = self.past_states[1][0::3][self.state_transitions_t['formation']:self.state_transitions_t['decoupling']]
            y_slave = self.past_states[1][1::3][self.state_transitions_t['formation']:self.state_transitions_t['decoupling']]
        else:
            raise NotImplementedError("Calculating the constraint violations for coupling and decoupling is not implemented")


        return [ ((x_master[i]-x_slave[i])**2 + (y_master[i]-y_slave[i])**2)**0.5 for i in range(len(x_master))]


    def plot(self, goal_positions):
        """Plots the traversed path and control signals of master and slave and constraint violations


        """
        start_time = perf_counter_ns()
       
        x_master = self.past_states[0][0::3]
        y_master = self.past_states[0][1::3]
        x_slave = self.past_states[1][0::3]
        y_slave = self.past_states[1][1::3]
        
        v_master = self.past_u[0][0::2]
        ang_master = self.past_u[0][1::2]
        v_slave = self.past_u[1][0::2]
        ang_slave = self.past_u[1][1::2]
        
        # constraint_deviation = [ ((x_master[i]-x_slave[i])**2 + (y_master[i]-y_slave[i])**2)**0.5 - self.solver_param.base.constraint['d'] for i in range(len(x_master))]
        constraint_deviation = self.calc_constraint_violation()
        
        time = np.linspace(0, self.solver_param.base.ts*(len(x_master)) - self.solver_param.base.ts, len(x_master))
        u_time = time[0:-1]
        

        # Velocity acceleration
        vel_acc_master = np.diff(v_master)/self.solver_param.base.ts
        vel_acc_slave = np.diff(v_slave)/self.solver_param.base.ts

        # ang acceleration
        ang_acc_master = np.diff(ang_master)/self.solver_param.base.ts
        ang_acc_slave = np.diff(ang_slave)/self.solver_param.base.ts


        # Velocity jerk
        vel_jerk_master = np.diff(vel_acc_master)/self.solver_param.base.ts
        vel_jerk_slave = np.diff(vel_acc_slave)/self.solver_param.base.ts

        # ang jerk
        ang_jerk_master = np.diff(ang_acc_master)/self.solver_param.base.ts
        ang_jerk_slave = np.diff(ang_acc_slave)/self.solver_param.base.ts


        # Plot map
        try:
            data = [x_master, y_master]
            self.plot_queues['master_path'].put_nowait(data)
            self.plot_queues['master_start'].put_nowait([[data[0][0]], [data[1][0]]])
            self.plot_queues['master_end'].put_nowait([[goal_positions[0][0]], [goal_positions[0][1]]])
            if self.plot_config['plot_slave']:
                self.plot_queues['slave_path'].put_nowait([x_slave, y_slave])
                self.plot_queues['slave_start'].put_nowait([[x_slave[0]], [y_slave[0]]])
                self.plot_queues['slave_end'].put_nowait([[goal_positions[1][0]], [goal_positions[1][1]]])

            # Plot data
            self.plot_queues['master_lin_vel'].put_nowait([u_time, v_master])
            self.plot_queues['master_ang_vel'].put_nowait([u_time, ang_master])
            if self.plot_config['plot_slave']:
                self.plot_queues['slave_lin_vel'].put_nowait([u_time, v_slave])
                self.plot_queues['slave_ang_vel'].put_nowait([u_time, ang_slave])

            if np.all(u_time[1:].shape):
                self.plot_queues['master_lin_acc'].put_nowait([u_time[1:], vel_acc_master])
                self.plot_queues['master_ang_acc'].put_nowait([u_time[1:], ang_acc_master])
                if self.plot_config['plot_slave']:
                    self.plot_queues['slave_lin_acc'].put_nowait([u_time[1:], vel_acc_slave])
                    self.plot_queues['slave_ang_acc'].put_nowait([u_time[1:], ang_acc_slave])

            if np.all(u_time[2:].shape):
                self.plot_queues['master_lin_jerk'].put_nowait([u_time[2:], vel_jerk_master])
                self.plot_queues['master_ang_jerk'].put_nowait([u_time[2:], ang_jerk_master])
                if self.plot_config['plot_slave']:
                    self.plot_queues['slave_lin_jerk'].put_nowait([u_time[2:], vel_jerk_slave])
                    self.plot_queues['slave_ang_jerk'].put_nowait([u_time[2:], ang_jerk_slave])

            if self.plot_config['plot_slave']:
                self.plot_queues['constraint'].put_nowait([time, constraint_deviation])
            
        
        except Full:
            pass

        self.times['plot'].append(perf_counter_ns() - start_time)
            
    def plot_trajectory(self, trajectory):
        """Plot predicted path and control inputs and constraint deviation

        Args:
            u (list): list of control inputs
        """
        start_time = perf_counter_ns()

        # Map
        x_master = trajectory[:, 0]
        y_master = trajectory[:, 1]
        theta_master = trajectory[:, 2]
        x_slave = trajectory[:, 3]
        y_slave = trajectory[:, 4]
        theta_slave = trajectory[:, 5]
        try:
            self.plot_queues['trajectory_master'].put_nowait([x_master, y_master])
            if self.plot_config['plot_slave']:
                self.plot_queues['trajectory_slave'].put_nowait([x_slave, y_slave])

            # This is temp. It's so that the plot doesn't get cluttered with look ahead
            self.plot_queues['trajectory_master'].put_nowait([x_master[:1], y_master[:1]])
            if self.plot_config['plot_slave']:
                self.plot_queues['trajectory_slave'].put_nowait([x_slave[:1], y_slave[:1]])
        except Full:
            pass


        self.times['plot'].append(perf_counter_ns() - start_time)

    