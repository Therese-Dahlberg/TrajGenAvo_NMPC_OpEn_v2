# Python imports
import os, sys, math
import traceback
from time import perf_counter_ns
from enum import Enum

import scipy
import scipy.spatial
import numpy as np

import shapely
from sympy import symbols, solve, lambdify

# Own imports
from ATRS import ATRS


class States(Enum):
    COUPLING = 0
    FORMATION = 1
    DECOUPLING = 2
    DONE = 3

    def transition(self, x, x_ref, atol=0.05, rtol=0.00001):
        return np.allclose(x, x_ref, atol=atol, rtol=rtol) 

    def next(self):
        if self == States.COUPLING:
            return States.FORMATION
        elif self == States.FORMATION:
            return States.DECOUPLING
        elif self == States.DECOUPLING:
            return States.DONE
        elif self == States.DONE:
            return None
        else:
            traceback.print_exc()
            raise(Exception("This state doesn't exist!"))

class PanocNMPCTrajectoryProblem: 
    def __init__(self, solver_param, solver, mpc_generator): 
        self.solver_param = solver_param
        self.solver = solver
        self.mpc_generator = mpc_generator

        self.initial_guess_pos = None

    def obstacle_as_inequality(self, Vs):
        # def vert2con(Vs):
        '''
        Compute the H-representation of a set of points (facet enumeration).
        Arguments:
            Vs: np.mat(np.array([[x0, y0], [x1, y1], [x2, y2]]))
        Returns:
            A   (L x d) array. Each row in A represents hyperplane normal.
            b   (L x 1) array. Each element in b represents the hyperpalne
                constant bi
        Taken from https://github.com/d-ming/AR-tools/blob/master/artools/artools.py
        '''
        hull = scipy.spatial.ConvexHull(Vs)
        K = hull.simplices
        c = np.mean(Vs[hull.vertices, :], 0)  # c is a (1xd) vector

        # perform affine transformation (subtract c from every row in Vs)
        V = Vs - c
        A = scipy.NaN * np.empty((K.shape[0], Vs.shape[1]))

        rc = 0
        for i in range(K.shape[0]):
            ks = K[i, :]
            F = V[ks, :]
            if np.linalg.matrix_rank(F) == F.shape[0]:
                f = np.ones(F.shape[0])
                A[rc, :] = scipy.linalg.solve(F, f)
                rc += 1

        A = A[0:rc, :]
        b = np.dot(A, c.T) + 1.0
        return (A, b)

    def obstacles_as_inequalities_and_vertices(self, obstacles):
        """
        Returns the obstacles in a list of lists of flattened b and a and v (= vx,vy): [[b0 a0^T v0^T] ... [bn an^T vn^T]]
        """
        obs_equations_and_vertices = []
        for ob in obstacles:
            # Get vertices of shapely object
            vert = np.vstack(ob.exterior.xy).T
            # Convert the obstacle into line inequalities and add them to the list
            A, b = self.obstacle_as_inequality(np.mat(vert.ravel().reshape(-1, 2)))
            obs_array = np.array(np.concatenate((b, A), axis = 1))
            # Clip last vertex that comes from shapely definition
            verts = vert[:-1]
            # Combine and reshape
            eqs_and_verts = np.concatenate((obs_array,verts), axis = 1).reshape(-1)
            obs_equations_and_vertices.append(eqs_and_verts)
        
        return obs_equations_and_vertices

    def obstacle_as_ellipse(self, obstacle):
        """Converts obstacles into ellipses with representation:  (x-c).T * A * (x-c) = 1.
        Returns A and c. Can convert into any dimension but is inteded to be used for 2D.
        
        Args:
            n_steps_ahead (int, optional): How many time steps ahead to be returned. Defaults to self.config.trajectory_length.

        Returns:
            list of np arrays: [np.array([A11 A12 A21 A22 c1 c2]), np.array([A11 A12 A21 A22 c1 c2])...]
        """
        start_time = perf_counter_ns()
        def mvee(points, tol = 0.001):
            """
            Find the minimum volume ellipse.
            Return A, c where the equation for the ellipse given in "center form" is
            (x-c).T * A * (x-c) = 1

            Taken from https://stackoverflow.com/questions/14016898/port-matlab-bounding-ellipsoid-code-to-python
            """
            points = np.asmatrix(points)
            N, d = points.shape
            Q = np.column_stack((points, np.ones(N))).T
            err = tol+1.0
            u = np.ones(N)/N
            while err > tol:
                # assert u.sum() == 1 # invariant
                X = Q * np.diag(u) * Q.T
                M = np.diag(Q.T * np.linalg.inv(X) * Q)
                jdx = np.argmax(M)
                step_size = (M[jdx]-d-1.0)/((d+1)*(M[jdx]-1.0))
                new_u = (1-step_size)*u
                new_u[jdx] += step_size
                err = np.linalg.norm(new_u-u)
                u = new_u
            c = u*points
            A = np.linalg.inv(points.T*np.diag(u)*points - c.T*c)/d    
            return np.asarray(A), np.squeeze(np.asarray(c))

        # Make sure the dtype is float. It sometimes is object
        obstacle = obstacle if obstacle.dtype==float else np.array(obstacle, dtype=float)
        ob = mvee(obstacle, tol=.05)
        return ob

    def obstacles_as_ellipses(self, obstacles):
        if not type(obstacles).__module__ == np.__name__: obstacles = np.array(obstacles)
        if obstacles.shape[1] == 0: return np.array([[]])

        obs = []
        for ob in obstacles:
            A, c = self.obstacle_as_ellipse(ob)
            obs.append(np.concatenate((np.reshape(A, -1), c)))
        return obs

    def convert_bounds_to_eqs(self, boundary_padded):
        bounds = boundary_padded
        A, B = self.obstacle_as_inequality(np.mat(bounds))
        bounds_array = np.array(np.concatenate((B, A), axis = 1)).reshape(-1)
        #convert to list
        bounds_array = list(bounds_array)
        n_params = len(bounds_array)
        req_param = self.solver_param.base.n_bounds_vertices*self.solver_param.base.n_param_line
        if n_params < req_param:
            extra_params = np.zeros(req_param-n_params)
            bounds_array = bounds_array + list(extra_params)
        return bounds_array

    def convert_static_obs_to_eqs_and_verts(self, closest_obs):
        """[summary]

        Args:
            x_cur ([type]): [description]
        
        Returns:
            obs_equations (numpy array): 1-d numpy array where the static and unexpected obstacles as [b0 a0^T ... bn an^T]
        
        """
        # Convert the closest static and dynamic obstacles into inequalities AND x,y coordinates of all vertices
        obs_equations = []
        n_param = self.solver_param.base.n_param_line + self.solver_param.base.n_param_vertex
        for key in closest_obs:     # keys are order of polygon
            obs = closest_obs[key]  # get list of static obs of order 'key'
            key_obs_equations_and_vertices = np.array(self.obstacles_as_inequalities_and_vertices(obs)).reshape(-1)  # Obs converted in 'key' half-spaces with A*x = b in R^'key'
            # check what happens with zeros of vertices
            # If there aren't enough obstacles, fill with 0s
            n_obs = len(obs) 
            if n_obs < self.solver_param.base.n_obs_of_each_vertices: #TODO: Add special case for triangles
                # obs_array = float('inf')*np.ones((self.solver_param.base.n_obs_of_each_vertices-n_obs)*n_param*key) # Set inf for equation params of every additional obs for each order
                obs_array = np.zeros((self.solver_param.base.n_obs_of_each_vertices-n_obs)*n_param*key)
                key_obs_equations_and_vertices = np.hstack((key_obs_equations_and_vertices, obs_array))
            obs_equations.append(key_obs_equations_and_vertices)
        
        obs_equations = np.hstack(obs_equations)  # Combine all equations of each order into one array

        # Make sure the correct number of params are sent
        assert(sum(range(self.solver_param.base.min_vertices, self.solver_param.base.max_vertices+1)) * n_param * self.solver_param.base.n_obs_of_each_vertices == sum(obs_equations.shape)), "An invalid amount of static obstacles parameters were sent."
        return obs_equations, closest_obs

    def convert_dynamic_obs_to_eqs(self, closest_obs):
        # Convert them into ellipses
        obs_ellipse = []
        for ob in closest_obs:
            obs_ellipse.append(self.obstacles_as_ellipses([np.array(ob_.exterior.xy).T[:-1] for ob_ in ob]))
        ellipse_to_plot = obs_ellipse.copy()

        # If there aren't enough obstacles, append 0s
        n_obs = len(obs_ellipse)
        obs_ellipse = np.array(obs_ellipse).reshape(-1)
        if n_obs < self.solver_param.base.n_dyn_obs:
            obs_array = np.zeros((self.solver_param.base.n_dyn_obs - n_obs) * self.solver_param.base.n_param_dyn_obs * self.solver_param.base.n_hor)
            obs_ellipse = np.hstack((obs_ellipse, obs_array))
    
        # Make sure the correct amount of parameters are returned
        assert(self.solver_param.base.n_param_dyn_obs *self.solver_param.base.n_hor*self.solver_param.base.n_dyn_obs == sum(obs_ellipse.shape)), "An invalid amount of dynamic obstacles parameters were sent."
        return obs_ellipse, closest_obs, ellipse_to_plot


    def get_active_dyn_obs(self, dyn_constraints):
        dyn_constraints_len = len(dyn_constraints)
        already_set = False
        active_obs = [1]*self.solver_param.base.n_dyn_obs
        a0_dyn = dyn_constraints[0: dyn_constraints_len: self.solver_param.base.n_param_dyn_obs]
        a1_dyn = dyn_constraints[1: dyn_constraints_len: self.solver_param.base.n_param_dyn_obs]
        a2_dyn = dyn_constraints[2: dyn_constraints_len: self.solver_param.base.n_param_dyn_obs]
        a3_dyn = dyn_constraints[3: dyn_constraints_len: self.solver_param.base.n_param_dyn_obs]
        c0_dyn = dyn_constraints[4: dyn_constraints_len: self.solver_param.base.n_param_dyn_obs]
        c1_dyn = dyn_constraints[5: dyn_constraints_len: self.solver_param.base.n_param_dyn_obs]
        for t in range(0, self.solver_param.base.n_hor):
            a0_t = a0_dyn[t :: self.solver_param.base.n_hor]
            a1_t = a1_dyn[t :: self.solver_param.base.n_hor]
            a2_t = a2_dyn[t :: self.solver_param.base.n_hor]
            a3_t = a3_dyn[t :: self.solver_param.base.n_hor]
            c0_t = c0_dyn[t :: self.solver_param.base.n_hor]
            c1_t = c1_dyn[t :: self.solver_param.base.n_hor]
            for obs in range(0, self.solver_param.base.n_dyn_obs):

                if a0_t[obs] == 0 and a1_t[obs] == 0 and a2_t[obs] == 0 and a3_t[obs] == 0 and c0_t[obs] == 0 and c1_t[obs] == 0:
                    active_obs[obs] = 0
        
        return active_obs

    def generate_control_input(self, parameters, guess=None):
        """Calls on the nmpc solver mng with parameters to find the next control inputs

        Args:
            mng (npmc solver): nmpc solver fit for the current task
            parameters (list): list of parameters. Specified in default.yaml

        Returns:
            list: control inputs for the atrs, for how many time steps is specififed by num_steps_taken in default.yaml. [v_master, ang_slave, v_slave, oemga_slave ....]
        """
        
        if guess is None:
            solution = self.solver.run(parameters)
        else:
            solution = self.solver.run(parameters, initial_guess=guess)

        exit_status = solution.exit_status
        u = solution.solution
        
        assert solution.cost >= 0, f"The cost cannot be negative. It was {solution.cost}"
        return u, solution.cost

    def gen_pos_traj(self, x_cur, x_finish, constraints, dyn_constraints, active_dyn_obs, distance_lb, distance_ub, bounds_eqs, u_previous, initial_guess_pos, prev_acc):
        d_near = 0.00001
        acc_init = prev_acc

        refs = x_finish * self.solver_param.base.n_hor
        u_ref = [0]*2*2*self.solver_param.base.n_hor

        if ((x_cur[0]-x_finish[0])**2 + (x_cur[1]-x_finish[1])**2) > d_near:
            q_master_denominator = ((x_cur[0]-x_finish[0])**2 + (x_cur[1]-x_finish[1])**2)
        else:
            q_master_denominator = 1
        if ((x_cur[3]-x_finish[3])**2 + (x_cur[4]-x_finish[4])**2) > d_near:
            q_slave_denominator = ((x_cur[3]-x_finish[3])**2 + (x_cur[4]-x_finish[4])**2)
        else:
            q_slave_denominator = 1

        weight_list = [self.solver_param.pos_goal_weights.q_lin_v, 
                self.solver_param.pos_goal_weights.q_lin_acc, 
                self.solver_param.pos_goal_weights.q_lin_ret, 
                self.solver_param.pos_goal_weights.q_lin_jerk, 
                self.solver_param.pos_goal_weights.q_ang, 
                self.solver_param.pos_goal_weights.q_ang_acc, 
                self.solver_param.pos_goal_weights.q_ang_jerk, 
                self.solver_param.pos_goal_weights.q_cte, 
                self.solver_param.pos_goal_weights.q_pos_master/q_master_denominator, 
                self.solver_param.pos_goal_weights.q_pos_slave*self.solver_param.base.enable_slave/q_slave_denominator,
                self.solver_param.pos_goal_weights.q_d_lin_vel_upper, 
                self.solver_param.pos_goal_weights.q_d_lin_vel_lower, 
                self.solver_param.pos_goal_weights.q_reverse, 
                self.solver_param.pos_goal_weights.q_d_ang_vel, 
                self.solver_param.pos_goal_weights.q_theta_master*np.allclose(np.array(x_cur)[[0,1]], np.array(x_finish)[[0,1]], atol=2, rtol=0.00001), 
                self.solver_param.pos_goal_weights.q_theta_slave*self.solver_param.base.enable_slave* np.allclose(np.array(x_cur)[[3,4]], np.array(x_finish)[[3,4]], atol=2, rtol=0.00001),
                self.solver_param.pos_goal_weights.q_pos_N/(min(q_master_denominator, q_slave_denominator)), 
                self.solver_param.pos_goal_weights.q_theta_N*np.allclose(np.array(x_cur)[[0,1,3,4]], np.array(x_finish)[[0,1,3,4]], atol=2, rtol=0.00001), 
                self.solver_param.pos_goal_weights.q_distance,
                self.solver_param.pos_goal_weights.q_theta_diff, 
                self.solver_param.pos_goal_weights.q_distance_c,
                self.solver_param.pos_goal_weights.q_obs_c, 
                self.solver_param.pos_goal_weights.q_dyn_obs_c, 
                self.solver_param.pos_goal_weights.q_future_dyn_obs, 
                self.solver_param.pos_goal_weights.q_acc_c, 
                self.solver_param.pos_goal_weights.enable_distance_constraint, 
                self.solver_param.pos_goal_weights.q_formation_ang,
                self.solver_param.pos_goal_weights.q_line_theta,
                ]
        
        
        parameters = x_cur + x_finish + u_previous + acc_init + [self.solver_param.base.constraint['d']] + [distance_lb] + [distance_ub] + [0] + [self.solver_param.base.constraint['formation_angle_error_margin']] + weight_list + u_ref + refs +  list(constraints) + list(dyn_constraints) + active_dyn_obs + bounds_eqs
        parameters = [float(param) for param in parameters]

        
        if initial_guess_pos is not None:
            # self.initial_guess_pos = self.initial_guess_pos[self.solver_param.base.nu:] + self.initial_guess_pos[-self.solver_param.base.nu:]
            initial_guess_pos = initial_guess_pos[self.solver_param.base.nu:] + [0]*self.solver_param.base.nu
            
        solution, ref_cost = self.generate_control_input(parameters, guess=initial_guess_pos)

        return ref_cost, parameters, solution

    def gen_line_following_traj(self, x_cur, dyn_constraints, line_vertices_master, line_vertices_slave, formation_angle_ref, constraints, active_dyn_obs, distance_lb, distance_ub, aggressive_factor, bounds_eqs, u_prev, initial_guess_master, prev_acc):
        master_angle_ref = self.mpc_generator.calc_master_ref_angle(x_cur[:3], *line_vertices_master[:3])

        # Build parameter list
        acc_init = [prev_acc[0], prev_acc[1], 0, 0] # We don't care about the slaves accelerations
        u_ref = [1,0,0,0] * self.solver_param.base.n_hor# * self.solver_param.base.nu #TODO: 1 shouldn't be hardcoded

        x_finish = list(line_vertices_master[-3]) + [master_angle_ref] + list(line_vertices_slave[-3]) + [0]
        # Modify the reference values so that theta_ref = 0 for both master and slave
        refs = []# This is a bad name. What it containts is the line vertices
        for i in range(0, self.solver_param.base.n_hor):
            refs += list(line_vertices_master[i]) + [0] 
            refs += list(line_vertices_slave[i]) + [0]
        
        # Master only trajectory generation weights
        weight_list_soft = [self.solver_param.line_follow_weights.q_lin_v, 
                    self.solver_param.line_follow_weights.q_lin_acc, 
                    self.solver_param.line_follow_weights.q_lin_ret, 
                    self.solver_param.line_follow_weights.q_lin_jerk, 
                    self.solver_param.line_follow_weights.q_ang, 
                    self.solver_param.line_follow_weights.q_ang_acc, 
                    self.solver_param.line_follow_weights.q_ang_jerk, 
                    self.solver_param.line_follow_weights.q_cte, 
                    self.solver_param.line_follow_weights.q_pos_master, 
                    self.solver_param.line_follow_weights.q_pos_slave, 
                    self.solver_param.line_follow_weights.q_d_lin_vel_upper, 
                    self.solver_param.line_follow_weights.q_d_lin_vel_lower, 
                    self.solver_param.line_follow_weights.q_reverse, 
                    self.solver_param.line_follow_weights.q_d_ang_vel, 
                    self.solver_param.line_follow_weights.q_theta_master, 
                    self.solver_param.line_follow_weights.q_theta_slave, 
                    self.solver_param.line_follow_weights.q_pos_N, 
                    self.solver_param.line_follow_weights.q_theta_N, 
                    self.solver_param.line_follow_weights.q_distance, 
                    self.solver_param.line_follow_weights.q_theta_diff, 
                    self.solver_param.line_follow_weights.q_distance_c, 
                    self.solver_param.line_follow_weights.q_obs_c, 
                    self.solver_param.line_follow_weights.q_dyn_obs_c, 
                    self.solver_param.line_follow_weights.q_future_dyn_obs, 
                    self.solver_param.line_follow_weights.q_acc_c, 
                    self.solver_param.line_follow_weights.enable_distance_constraint, 
                    self.solver_param.line_follow_weights.q_formation_ang,
                    self.solver_param.line_follow_weights.q_line_theta,
                    ]
        # Put aggressive weights into a list, 
        # Same outcome as code above, 
        weight_list_aggressive = []
        for weight in self.solver_param.line_follow_weights_aggressive:
            if not weight == 'n_weights': # dont include this
                weight_list_aggressive.append(self.solver_param.line_follow_weights_aggressive[weight])

        weight_list = []
        # Merge the two lists, if aggressive_factor is 0 => new list is equal to weight_list_soft
        # if aggresive_factor = 1 => new list is equal to weight_list_aggressive
        # between 0 and 1 weight list is an interpolation of the two.
        assert aggressive_factor <= 1 and aggressive_factor >= 0, " aggressive_factor must be a number between 0 and 1" 
        for i in range(len(weight_list_aggressive)):
            mixed_weight = weight_list_soft[i]*(1-aggressive_factor) + weight_list_aggressive[i] * aggressive_factor
            weight_list.append( mixed_weight)

        u_previous = u_prev[:2] + [0,0]
        parameters = x_cur + x_finish + u_previous + acc_init + [self.solver_param.base.constraint['d']] + [distance_lb] + [distance_ub] + [formation_angle_ref] + [self.solver_param.base.constraint['formation_angle_error_margin']] + weight_list + u_ref + refs +  list(constraints) + list(dyn_constraints) + active_dyn_obs + bounds_eqs

        parameters = [float(param) for param in parameters]

        #store input parameters for simulator plots
        if initial_guess_master is not None:
            # self.initial_guess_master = self.initial_guess_master[self.solver_param.base.nu:] + self.initial_guess_master[-self.solver_param.base.nu:]
            initial_guess_master = initial_guess_master[self.solver_param.base.nu:] + [0]*self.solver_param.base.nu
        solution, ref_cost = self.generate_control_input(parameters, guess=initial_guess_master)
                                
        
        return ref_cost, parameters, solution

    def gen_traj_following_traj(self, trajectory_ref, u_ref, x_cur, formation_angle_ref, x_finish, constraints, dyn_constraints, active_dyn_obs, distance_lb, distance_ub, aggressive_factor, bounds_eqs, u_previous, initial_guess_dual, prev_acc):
        acc_init = prev_acc
        
        x_ref_master = trajectory_ref[0::6]
        y_ref_master = trajectory_ref[1::6]
        block_dist_master = np.vstack((np.array(x_ref_master)-x_cur[0], np.array(y_ref_master)-x_cur[1]))
        dist_to_refs_master = np.linalg.norm(block_dist_master,axis=0)
        avg_dist_to_ref_master = np.mean(dist_to_refs_master)

        x_ref_slave = trajectory_ref[3::6]
        y_ref_slave = trajectory_ref[4::6]
        block_dist_slave = np.vstack((np.array(x_ref_slave)-x_cur[0], np.array(y_ref_slave)-x_cur[1]))
        dist_to_refs_slave = np.linalg.norm(block_dist_slave,axis=0)
        avg_dist_to_ref_slave = np.mean(dist_to_refs_slave)

        weight_list = []
        weight_list_soft = [self.solver_param.traj_follow_weights.q_lin_v, 
                self.solver_param.traj_follow_weights.q_lin_acc, 
                self.solver_param.traj_follow_weights.q_lin_ret, 
                self.solver_param.traj_follow_weights.q_lin_jerk, 
                self.solver_param.traj_follow_weights.q_ang, 
                self.solver_param.traj_follow_weights.q_ang_acc, 
                self.solver_param.traj_follow_weights.q_ang_jerk, 
                self.solver_param.traj_follow_weights.q_cte, 
                self.solver_param.traj_follow_weights.q_pos_master/avg_dist_to_ref_master, 
                self.solver_param.traj_follow_weights.q_pos_slave/avg_dist_to_ref_slave,
                self.solver_param.traj_follow_weights.q_d_lin_vel_upper, 
                self.solver_param.traj_follow_weights.q_d_lin_vel_lower, 
                self.solver_param.traj_follow_weights.q_reverse, 
                self.solver_param.traj_follow_weights.q_d_ang_vel, 
                self.solver_param.traj_follow_weights.q_theta_master, 
                self.solver_param.traj_follow_weights.q_theta_slave * (1 if np.abs(x_cur[-1]-trajectory_ref[-1]) < np.pi else 50), #The increase in cost should probably be changed to a smooth function of some kind
                self.solver_param.traj_follow_weights.q_pos_N, 
                self.solver_param.traj_follow_weights.q_theta_N, 
                self.solver_param.traj_follow_weights.q_distance,
                self.solver_param.traj_follow_weights.q_theta_diff, 
                self.solver_param.traj_follow_weights.q_distance_c,
                self.solver_param.traj_follow_weights.q_obs_c, 
                self.solver_param.traj_follow_weights.q_dyn_obs_c, 
                self.solver_param.traj_follow_weights.q_future_dyn_obs, 
                self.solver_param.traj_follow_weights.q_acc_c, 
                self.solver_param.traj_follow_weights.enable_distance_constraint, 
                self.solver_param.traj_follow_weights.q_formation_ang,
                self.solver_param.traj_follow_weights.q_line_theta,
                ]
     
        weight_list_aggressive = [self.solver_param.traj_follow_weights_aggressive.q_lin_v, 
                self.solver_param.traj_follow_weights_aggressive.q_lin_acc, 
                self.solver_param.traj_follow_weights_aggressive.q_lin_ret, 
                self.solver_param.traj_follow_weights_aggressive.q_lin_jerk, 
                self.solver_param.traj_follow_weights_aggressive.q_ang, 
                self.solver_param.traj_follow_weights_aggressive.q_ang_acc, 
                self.solver_param.traj_follow_weights_aggressive.q_ang_jerk, 
                self.solver_param.traj_follow_weights_aggressive.q_cte, 
                self.solver_param.traj_follow_weights_aggressive.q_pos_master/avg_dist_to_ref_master, 
                self.solver_param.traj_follow_weights_aggressive.q_pos_slave/avg_dist_to_ref_slave,
                self.solver_param.traj_follow_weights_aggressive.q_d_lin_vel_upper, 
                self.solver_param.traj_follow_weights_aggressive.q_d_lin_vel_lower, 
                self.solver_param.traj_follow_weights_aggressive.q_reverse, 
                self.solver_param.traj_follow_weights_aggressive.q_d_ang_vel, 
                self.solver_param.traj_follow_weights_aggressive.q_theta_master, 
                self.solver_param.traj_follow_weights_aggressive.q_theta_slave * (1 if np.abs(x_cur[-1]-trajectory_ref[-1]) < np.pi else 50), #The increase in cost should probably be changed to a smooth function of some kind
                self.solver_param.traj_follow_weights_aggressive.q_pos_N, 
                self.solver_param.traj_follow_weights_aggressive.q_theta_N, 
                self.solver_param.traj_follow_weights_aggressive.q_distance,
                self.solver_param.traj_follow_weights_aggressive.q_theta_diff, 
                self.solver_param.traj_follow_weights_aggressive.q_distance_c,
                self.solver_param.traj_follow_weights_aggressive.q_obs_c, 
                self.solver_param.traj_follow_weights_aggressive.q_dyn_obs_c, 
                self.solver_param.traj_follow_weights_aggressive.q_future_dyn_obs, 
                self.solver_param.traj_follow_weights_aggressive.q_acc_c, 
                self.solver_param.traj_follow_weights_aggressive.enable_distance_constraint, 
                self.solver_param.traj_follow_weights_aggressive.q_formation_ang,
                self.solver_param.traj_follow_weights_aggressive.q_line_theta,
                ]
        
        # Merge the two lists, if aggressive_factor is 0 => new list is equal to weight_list_soft
        # if aggresive_factor = 1 => new list is equal to weight_list_aggressive
        # between 0 and 1 weight list is an interpolation of the two.
        assert aggressive_factor <= 1 and aggressive_factor >= 0, " aggressive_factor must be a number between 0 and 1" 
        for i in range(len(weight_list_aggressive)):
            mixed_weight = weight_list_soft[i]*(1-aggressive_factor) + weight_list_aggressive[i] * aggressive_factor
            weight_list.append( mixed_weight)

        parameters = x_cur + x_finish + u_previous + acc_init + [self.solver_param.base.constraint['d']] + [distance_lb] + [distance_ub] + [formation_angle_ref] + [self.solver_param.base.constraint['formation_angle_error_margin']] + weight_list + u_ref + trajectory_ref +  list(constraints) + list(dyn_constraints) + active_dyn_obs  + bounds_eqs
        parameters = [float(param) for param in parameters]

        
        if initial_guess_dual is not None:
            # self.initial_guess_dual = self.initial_guess_dual[self.solver_param.base.nu:] + self.initial_guess_dual[-self.solver_param.base.nu:]
            initial_guess_dual = initial_guess_dual[self.solver_param.base.nu:] + [0]*self.solver_param.base.nu
            
        solution, ref_cost = self.generate_control_input(parameters, initial_guess_dual)
        

        return ref_cost, parameters, solution

    def calculate_dist_between_nodes(self, nodes, cumsum=True):
        assert(nodes.shape[1]==2), f"The nodes array sent to {self.print_name}.calculate_dist_between_nodes must be of shape: (-1, 2). They were in shape: {nodes.shape}."
        block_distance = np.diff(nodes, axis=0)
        euclidian_distance = np.linalg.norm(block_distance, axis=1)
        
        if cumsum: return np.cumsum(euclidian_distance)
        else: return euclidian_distance

    def generate_refs(self, x_cur, ref_points):
        """[summary]

        Args:
            x_cur ([type]): [description]
            ref_points ([type]): [description]

        Returns:
            [best_ref_point]: [Reachable ref point]
            [local_ref_points]: [n_hor points close to current]
        """        
        # Find the best ref point
        dist_to_nodes = np.linalg.norm(np.array(ref_points) - np.array(x_cur[:2]), axis=1)
        closest_idx = np.argmin(dist_to_nodes)

        nodes_forward = ref_points[closest_idx:]
        dist_to_nodes_forward = self.calculate_dist_between_nodes(np.array(nodes_forward))
        dist_to_nodes_forward -= 1 #self.solver_param.base.lin_vel_max*self.solver_param.base.ts*self.solver_param.base.n_hor
        
        ref_point_within_reach = False
        # if all of the nodes are within reach:
        if not np.all(dist_to_nodes_forward[dist_to_nodes_forward > 1e-3].shape):
            best_reference_point_idx = -1
            best_reference_point = list(ref_points[best_reference_point_idx])
            ref_point_within_reach = True

        else:
            best_dist = np.min(dist_to_nodes_forward[dist_to_nodes_forward > 1e-3]) #1e-3 is just an epsilon
            best_reference_point_idx = np.where(dist_to_nodes_forward == best_dist)[0][0] + closest_idx

            best_reference_point = ref_points[best_reference_point_idx]

        best_reference_point = list(best_reference_point) + [0]
            
        # Find the line vertices; Find the first vertix on the current line
        cur_node = ref_points[0]
        node_to_check_idx = 1
        calc_ang_between_nodes = lambda p1, p2: np.arctan2(p2[1]-p1[1], p2[0]-p1[0])
        line_vertices = [cur_node]

        # Find the following veunexpectedrtices
        while node_to_check_idx < len(ref_points):
            cur_node = line_vertices[-1]
            ang_between_nodes = calc_ang_between_nodes(cur_node, ref_points[node_to_check_idx])
            
            for node in ref_points[node_to_check_idx:]:
                if ang_between_nodes == calc_ang_between_nodes(cur_node, node):
                    node_to_check_idx += 1
                else:
                    line_vertices.append(ref_points[node_to_check_idx-1])
                    break

        # Add last vertix
        line_vertices.append(ref_points[-1])

        # Calculate closest lines
        lines = [[np.array(line_vertices[i], dtype=float), np.array(line_vertices[i+1], dtype=float)] for i in range(len(line_vertices)-1)]
        dist_to_lines = np.array([self.mpc_generator.dist_to_line(x_cur[:2], *line).__float__() for line in lines])**0.5
        closest_line_idx = np.argmin(dist_to_lines)

        # Find the first vertix on the current line
        line_vertices = [lines[closest_line_idx][0]]
        dist_lines_ahead = np.cumsum(dist_to_lines[closest_line_idx:]) - self.solver_param.base.dist_to_start_turn*100 
        for i in range(len(lines[closest_line_idx:])): 
            if dist_lines_ahead[i] < 0: 
                line_vertices.append(lines[i+closest_line_idx][1]) 

        # Make sure the list of vertices is long enough
        line_vertices += [line_vertices[-1]]*(self.solver_param.base.n_hor - len(line_vertices))

        assert(len(line_vertices) == self.solver_param.base.n_hor), f"There are too many ref points. There can only be {self.solver_param.base.n_hor} and there were {len(line_vertices)}."
                
        return best_reference_point, line_vertices, ref_point_within_reach

    def offset_trajectory(self, base_trajectory, offset_angle):
        offset_traj = []
        for t in range(self.solver_param.base.n_hor):
            master_ref = base_trajectory[t, :3]
            slave_ref = master_ref[:2] + np.array([np.cos(offset_angle)*self.solver_param.base.constraint['d'], np.sin(offset_angle)*self.solver_param.base.constraint['d']])

            # Calculate the angle the slave should be in
            slave_prev = base_trajectory[0, :3] + np.array([np.cos(offset_angle)*self.solver_param.base.constraint['d'], np.sin(offset_angle)*self.solver_param.base.constraint['d'], 0]) if t == 0 else base_trajectory[t-1, :3] + np.array([np.cos(offset_angle)*self.solver_param.base.constraint['d'], np.sin(offset_angle)*self.solver_param.base.constraint['d'], 0])
            slave_next = base_trajectory[1, :3] + np.array([np.cos(offset_angle)*self.solver_param.base.constraint['d'], np.sin(offset_angle)*self.solver_param.base.constraint['d'], 0]) if t == 0 else base_trajectory[-1, :3] + np.array([np.cos(offset_angle)*self.solver_param.base.constraint['d'], np.sin(offset_angle)*self.solver_param.base.constraint['d'], 0]) if t == self.solver_param.base.n_hor-1 else base_trajectory[t, :3] + np.array([np.cos(offset_angle)*self.solver_param.base.constraint['d'], np.sin(offset_angle)*self.solver_param.base.constraint['d'], 0])
            theta_slave = np.arctan2(slave_next[1]-slave_prev[1], slave_next[0]-slave_prev[0])  
            
            if theta_slave < 0 and slave_prev[-1] > np.pi/2:
                theta_slave += 2*np.pi
            elif theta_slave > 0 and slave_prev[-1] < -np.pi/2:
                theta_slave -= 2*np.pi

            slave_ref = np.hstack((slave_ref, theta_slave))
            offset_traj += list(master_ref) + list(slave_ref)

        #Make sure all distances diff is d
        assert  1e-5 > self.solver_param.base.constraint['d'] - np.sum([(np.array(offset_traj[i::6]) - np.array(offset_traj[3+i::6]))**2 for i in range(2)])/self.solver_param.base.n_hor

        return offset_traj

    def calculate_trajectory(self, u, master_state, slave_state, plot_config):
        #TODO: do this in a more efficient way
        atr = ATRS(master_state, slave_state, self.solver_param, plot_config, False)
        [atr.update_pos(u[i*4:i*4+4], None, None) for i in range(len(u)//4)]
        x_master, y_master, theta_master, x_slave, y_slave, theta_slave = atr.get_prev_states()
        v_master, ang_master, v_slave, ang_slave = atr.get_prev_control_signals()

        x_master = np.array(x_master[1:], ndmin=2).transpose()
        y_master = np.array(y_master[1:], ndmin=2).transpose()
        theta_master = np.array(theta_master[1:], ndmin=2).transpose()

        x_slave = np.array(x_slave[1:], ndmin=2).transpose()
        y_slave = np.array(y_slave[1:], ndmin=2).transpose()
        theta_slave = np.array(theta_slave[1:], ndmin=2).transpose()

        v_master = np.array(v_master, ndmin=2).transpose()
        ang_master = np.array(ang_master, ndmin=2).transpose()

        v_slave = np.array(v_slave, ndmin=2).transpose()
        ang_slave = np.array(ang_slave, ndmin=2).transpose()

        trajectory = np.concatenate((x_master, y_master, theta_master, x_slave, y_slave, theta_slave, v_master, ang_master, v_slave, ang_slave), axis=1)

        return trajectory
        
