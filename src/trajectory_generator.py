# Python imports
import os, sys, yaml
from time import perf_counter_ns
from pathlib import Path
from queue import Full

import numpy as np

# Own imports
from ATRS import ATRS
from mpc.mpc_generator import MpcModule, get_length

from path_planner.path_planner import PathPlanner
from path_planner.obstacle_handler import ObstacleHandler

from panoc_nmpc_trajectory_problem import States
from panoc_nmpc_trajectory_problem import PanocNMPCTrajectoryProblem

from utils.plotter import start_plotter
from utils.config import Configurator, SolverParams, Weights


class TrajectoryGenerator:
    """Class that generates control inputs. Uses solver_paramuration file in solver_params folder
    """
    def __init__(self, start_pose, verbose=False, name="[TrajGen]", self_destruct=True, master_goal = [], slave_goal = [], build_solver=False):
        # Get all the setups
        solver_param, plot_queues, plot_process, plot_config = self.load_params()
        # Check if we should rebuild solver
        if build_solver:
            MpcModule(solver_param).build()
        # Setup plot stuff
        self.plot_config = plot_config
        self.plot_queues = plot_queues
        self.plot_process = plot_process
        self.self_destruct = self_destruct

        self.verbose = verbose 

        self.times = {'loop_time':[], 'solver':[], 'static_to_eq':[], 'dyn_to_eq':[], 'plot':[]}
        self.solver_times = []
        self.overhead_times = []
        self.print_name = name

        self.costs        = {key : [] for key in self.plot_queues if 'cost' in key and not 'future' in key}
        self.costs_future = {key : [] for key in self.plot_queues if 'cost' in key and 'future' in key}
        
        # Save solver parameters
        self.solver_param = solver_param

        # Import the solver
        sys.path.append(Path(__file__).parent.parent.__str__() + '/mpc_build/' + self.solver_param.base.optimizer_name+'/')
        import trajectory_generator_solver
        self.solver = trajectory_generator_solver.solver()

        # Setup all the classes that are needed
        self.robot_state = start_pose[0] + start_pose[1] # (start_master, start_slave)

        self.robot_data = ATRS(start_pose[0], start_pose[1], solver_param, plot_config, plot_queues)
        self.obs_handler = ObstacleHandler(solver_param.base, plot_config, plot_queues)   
        self.path_planner = PathPlanner(solver_param.base, plot_config, plot_queues)  
        self.mpc_generator = MpcModule(self.solver_param)
        self.panoc = PanocNMPCTrajectoryProblem(solver_param, self.solver, self.mpc_generator)

        # Some variables to keep track of the trajectory generator state
        self.initial_guess_master = None
        self.initial_guess_dual = None
        self.initial_guess_pos = None
        self.u_previous = [0]*self.solver_param.base.nu
        self.state = States.COUPLING

        # Variables for keeping track of plotting
        self.u_master_prev = []
        self.u_slave_prev = []
        self.v_master = []
        self.ang_master = []
        self.v_slave = []
        self.ang_slave = []
        
        # Goals
        self.master_goal = master_goal
        self.slave_goal = slave_goal
        self.master_end = list(self.master_goal[self.state.value])
        self.slave_end = list(self.slave_goal[self.state.value])

        # Flag for genereating 2s horizon
        self.two_seconds = False

        self.gen_traj = [[],[]]

    def load_params(self): 
        file_path = Path(__file__)

        base_fn = 'base.yaml'
        yaml_fp = os.path.join(str(file_path.parent.parent), 'configs', base_fn)
        configurator = Configurator(yaml_fp)
        base = configurator.configurate()

        weights_fn = 'line_follow_weights.yaml'
        yaml_fp = os.path.join(str(file_path.parent.parent), 'configs', weights_fn)
        configurator = Weights(yaml_fp)
        line_follow_weights = configurator.configurate()

        weights_fn = 'line_follow_weights_aggressive.yaml'
        yaml_fp = os.path.join(str(file_path.parent.parent), 'configs', weights_fn)
        configurator = Weights(yaml_fp)
        line_follow_weights_aggressive = configurator.configurate()

        weights_fn = 'traj_follow_weights.yaml'
        yaml_fp = os.path.join(str(file_path.parent.parent), 'configs', weights_fn)
        configurator = Weights(yaml_fp)
        traj_follow_weights = configurator.configurate()

        weights_fn = 'traj_follow_weights_aggressive.yaml'
        yaml_fp = os.path.join(str(file_path.parent.parent), 'configs', weights_fn)
        configurator = Weights(yaml_fp)
        traj_follow_weights_aggressive = configurator.configurate()

        weights_fn = 'pos_goal_weights.yaml'
        yaml_fp = os.path.join(str(file_path.parent.parent), 'configs', weights_fn)
        configurator = Weights(yaml_fp)
        pos_goal_weights = configurator.configurate()

        solver_param = SolverParams(base, line_follow_weights, traj_follow_weights, pos_goal_weights, line_follow_weights_aggressive, traj_follow_weights_aggressive)
        
        # Load plot config file
        plot_config = 'plot_config.yaml'
        plot_config_fp = os.path.join(str(file_path.parent.parent), 'configs', plot_config)
        with open(plot_config_fp) as f:
            plot_config  = yaml.load(f, Loader=yaml.FullLoader)
        plot_queues, plot_process = start_plotter(solver_param.base, plot_config, aut_test_config=None)

        return solver_param, plot_queues, plot_process, plot_config
        
    def set_global_path(self, path): 
        self.path_planner.set_global_path(path)
            

    def run(self, plot=False):
        """The main function for moving from start to end position
        """
        done = False
        # Generate one full trajectory
        generated_trajectory = [[],[]]    
        
        while not done: 
            x = self.robot_state

            # Local path planner, plans around unexpected obstacles
            _, static_padded_obs, _, _, unexpected_padded_obs, unexpected_obstacles_shapely = self.obs_handler.get_static_obstacles()
            boundry = self.obs_handler.get_boundry()

            self.path_planner.local_path_plan(x[:3], unexpected_padded_obs, unexpected_obstacles_shapely, static_padded_obs, boundry)

            # If the slave is disabled we shouldn't try to trajectory plan for it
            if not self.solver_param.base.enable_slave:
                self.slave_end = list(x[3:])
   
            t = 0
            start_time = perf_counter_ns()
            x_cur = list(x) # I think this means cannot plan all the way to goal since always get the current position

            # Get the closest obstacles, update_dynamic_obstacles() only used for simulatin
            self.obs_handler.update_dynamic_obstacles()
            closest_obs_static  = self.obs_handler.get_closest_static_obstacles(x_cur[:2])
            closest_obs_dynamic = self.obs_handler.get_closest_dynamic_obstacles(x_cur[:2])
            
            # Generate obstacle constraints in Panoc terms
            constraints, closest_static_unexpected_obs = self.panoc.convert_static_obs_to_eqs(closest_obs_static)
            dyn_constraints, closest_dynamic_obs_poly, closest_dynamic_obs_ellipse = list(self.panoc.convert_dynamic_obs_to_eqs(closest_obs_dynamic))
            active_dyn_obs = self.panoc.get_active_dyn_obs(dyn_constraints)
            bounds_eqs = self.panoc.convert_bounds_to_eqs(boundry)

            # Generate reference values
            x_finish_master, line_vertices_master, goal_in_reach = self.panoc.generate_refs(x_cur, self.path_planner.path)
            slave_path = self.path_planner.path[0:-1] + [(self.slave_end[0], self.slave_end[1])]
            x_finish_slave, line_vertices_slave, _ = self.panoc.generate_refs(x_cur[3:], slave_path)
            # Calculate reference angles
            formation_angle_ref = self.mpc_generator.calc_formation_angle_ref(x_cur[:3], *line_vertices_master[:3])
            ref_trajectory = None

            # Get previous acceleration
            if len(self.u_master_prev) < 4:
                prev_acc = [0,0,0,0]
            else: 
                master_lin_acc = (self.u_master_prev[0]-self.u_master_prev[2]) / self.solver_param.base.ts
                master_ang_acc = (self.u_master_prev[1]-self.u_master_prev[3]) / self.solver_param.base.ts
                slave_lin_acc = (self.u_slave_prev[0]-self.u_slave_prev[2]) / self.solver_param.base.ts
                slave_ang_acc = (self.u_slave_prev[1]-self.u_slave_prev[3]) / self.solver_param.base.ts
                prev_acc = [master_lin_acc, master_ang_acc, slave_lin_acc, slave_ang_acc]
            
            '''
            Running starts here
            '''
            # Calculate trajectory when not in formation
            if self.state == States.COUPLING or self.state == States.DECOUPLING:
                ref_cost, parameters, solution = self.panoc.gen_pos_traj(x_cur, self.master_end + self.slave_end, constraints, dyn_constraints, active_dyn_obs,  self.solver_param.base.vehicle_margin, 1e100, bounds_eqs, self.u_previous, self.initial_guess_pos, prev_acc)
                self.initial_guess_pos = solution
            # Calculate trajectory when in formation
            elif self.state == States.FORMATION: 
                # If goal is not in reach, follow the lines
                if not goal_in_reach:
                    # Generate one trajectory for master
                    ref_cost, parameters, solution = self.panoc.gen_line_following_traj(x_cur, dyn_constraints, line_vertices_master, line_vertices_slave, formation_angle_ref, constraints, active_dyn_obs, self.solver_param.base.constraint['distance_lb'], self.solver_param.base.constraint['distance_ub'], self.solver_param.base.aggressive_factor, bounds_eqs, self.u_previous, self.initial_guess_master, prev_acc)
                    self.initial_guess_master = solution
                    
                    if self.solver_param.base.enable_slave:
                        # Offset master trajectory to receive slave trajectory
                        trajectory = self.panoc.calculate_trajectory(solution[:2*2*self.solver_param.base.trajectory_length], x_cur[:3], x_cur[3:], self.plot_config)
                        ref_trajectory = self.panoc.offset_trajectory(trajectory, formation_angle_ref)
                        
                        ref_cost, parameters, solution = self.panoc.gen_traj_following_traj(ref_trajectory, solution, x_cur, formation_angle_ref, x_finish_master + x_finish_slave, constraints, dyn_constraints, active_dyn_obs, self.solver_param.base.constraint['distance_lb'], self.solver_param.base.constraint['distance_ub'], self.solver_param.base.aggressive_factor, bounds_eqs, self.u_previous, self.initial_guess_dual, prev_acc)
                        self.initial_guess_dual = solution
                # If goal is in reach, go to the goal
                elif goal_in_reach:
                    if self.solver_param.base.enable_slave:
                        ref_cost, parameters, solution = self.panoc.gen_pos_traj(x_cur, self.master_end + self.slave_end, constraints, dyn_constraints, active_dyn_obs, self.solver_param.base.constraint['distance_lb'], self.solver_param.base.constraint['distance_ub'], bounds_eqs, self.u_previous, self.initial_guess_pos, prev_acc)
                        self.initial_guess_pos = solution
                    else:
                        ref_cost, parameters, solution = self.panoc.gen_pos_traj(x_cur, self.master_end + self.slave_end, constraints, dyn_constraints, active_dyn_obs, self.solver_param.base.vehicle_margin, 1e100, bounds_eqs, self.u_previous, self.initial_guess_pos, prev_acc)
                        self.initial_guess_pos = solution
            else:
                raise ValueError(f"Invalid state to be in: {self.state}!")

            # Save control inputs u = [u_master, u_slave] = [v_master, omega_master, v_slave, omega_slave], 
            self.u_previous = solution[:self.solver_param.base.nu]
            self.mpc_generator.store_params(parameters)
            
            # From the control inputs calculate the trajectory using discretized robot model
            trajectory = self.panoc.calculate_trajectory(solution[:2*2*self.solver_param.base.trajectory_length], x_cur[:3], x_cur[3:], self.plot_config)
            # Update the state to the last position in the trajectory 
            if self.two_seconds: 
                self.robot_state = list(trajectory[-1][0:3]) + list(trajectory[-1][3:6])
            else: 
                self.robot_state = list(trajectory[0][0:3])  + list(trajectory[0][3:6])
            
            # Plot
            if plot:
                self.u_master_prev.extend(self.u_previous[0:2])
                self.u_slave_prev.extend(self.u_previous[2:4])
                if len(self.u_master_prev) > 4: 
                    self.u_master_prev.pop(0)
                    self.u_master_prev.pop(0)
                if len(self.u_slave_prev) > 4: 
                    self.u_slave_prev.pop(0)
                    self.u_slave_prev.pop(0)
                self.plot(trajectory, self.u_master_prev, self.u_slave_prev, generated_trajectory,solution[:2*2*self.solver_param.base.trajectory_length])
                #time.sleep(0.5)

            # Update the generated trajectory with the recently generated trajectory
            if self.two_seconds: 
                [generated_trajectory[0].append(pose) for pose in trajectory[:,0:3]]
            else:
                generated_trajectory[0].append(trajectory[0,0:3])
            
            if self.solver_param.base.enable_slave:
                if self.two_seconds: 
                    [generated_trajectory[1].append(pose) for pose in trajectory[:,3:6]]
                else: 
                    generated_trajectory[1].append(trajectory[0,3:6])
              

            t += self.solver_param.base.num_steps_taken
            self.times['loop_time'].append(perf_counter_ns() - start_time)

            # Check if done with current state
            done_with_state = self.state.transition(x, self.master_end + self.slave_end)
            if done_with_state:
                # TODO: This should be published and we'll receive a command to move to the next state
                if self.verbose:
                    print(f"{self.print_name} Finished with state: {self.state.name}, moving on to state: {self.state.next().name}")
                self.state = self.state.next() 

                # Check if done
                if self.state == States.DONE:
                    done = True
                else:
                    # Update current end positions
                    self.master_end = list(self.master_goal[self.state.value])
                    self.slave_end = list(self.slave_goal[self.state.value])

            if done:
                print("Finished the trajectory planning")
                if self.self_destruct:
                    self.kill()
                self.gen_traj = generated_trajectory
                
    # Returns the generated trajectory for the master and slave
    def getGeneratedTrajectory(self):
        return self.gen_traj
        
    
    def plot(self, trajectory, u_master_prev, u_slave_prev, past_trajectory, u):
        
        # Call to plot whatever the path planner wants to plot
        self.path_planner.plot()

        # Get the obstacles and plot that stuff
        static_original_obs, static_padded_obs, _, unexpected_original_obs, unexpected_padded_obs,_ = self.obs_handler.get_static_obstacles()
        dynamic_original_obs, dynamic_padded_obs, _ = self.obs_handler.get_dynamic_obstacles()
        boundry = self.obs_handler.get_boundry()
        self.obs_handler.plot(boundry, static_original_obs,static_padded_obs, unexpected_original_obs, unexpected_padded_obs, dynamic_original_obs, dynamic_padded_obs, self.panoc) 
        
        # Plot the closest static and unexpected obstacles
        start_time = perf_counter_ns()

        # Plot the past trajectory, start and goal positions
        past_traj_master = past_trajectory[0]
        past_traj_slave = past_trajectory[1]

        if len(past_trajectory[0]) == 0: 
            start_master = trajectory[0,0:2]
            if self.solver_param.base.enable_slave and self.plot_config['plot_slave']:
                start_slave = trajectory[0,3:5]
        else: 
            start_master = past_traj_master[0][0:2]
            if self.solver_param.base.enable_slave and self.plot_config['plot_slave']:
                start_slave = past_traj_slave[0][0:2]
        
        goal_positions = [self.master_goal[-1][:2], self.slave_goal[-1][:2]]

        try:
            data = [[x[0] for x in past_traj_master[:]], [y[1] for y in past_traj_master[:]]]
            self.plot_queues['master_path'].put_nowait(data)
            self.plot_queues['master_start'].put_nowait([[start_master[0]], [start_master[1]]])
            self.plot_queues['master_end'].put_nowait([[goal_positions[0][0]], [goal_positions[0][1]]])
            if self.solver_param.base.enable_slave and self.plot_config['plot_slave']:
                data = [[x[0] for x in past_traj_slave[:]], [y[1] for y in past_traj_slave[:]]]
                self.plot_queues['slave_path'].put_nowait(data)
                self.plot_queues['slave_start'].put_nowait([[start_slave[0]], [start_slave[1]]])
                self.plot_queues['slave_end'].put_nowait([[goal_positions[1][0]], [goal_positions[1][1]]])
        except Full:
            pass
        try:
            self.plot_queues['object'].put_nowait(((past_traj_master[-1][0], past_traj_slave[-1][0]),
                                                   (past_traj_master[-1][1], past_traj_slave[-1][1])))
        except:
            pass
        # Plot planned trajectory
        traj_master = trajectory[:,0:3]
        if self.solver_param.base.enable_slave and self.plot_config['plot_slave']:
            traj_slave = trajectory[:,3:6]
        try:
            data = [[x for x in traj_master[:,0]], [y for y in traj_master[:,1]]]
            self.plot_queues['planned_trajectory_master'].put_nowait(data)
            if self.solver_param.base.enable_slave and self.plot_config['plot_slave']:
                data = [[x for x in traj_slave[:,0]], [y for y in traj_slave[:,1]]]
                self.plot_queues['planned_trajectory_slave'].put_nowait(data)
        except Full:
            pass

        # Plot velocities and accelerations
        if self.two_seconds: 
            self.v_master.extend(u[0::4])
            self.ang_master.extend(u[1::4])
            self.v_slave.extend(u[2::4])
            self.ang_slave.extend(u[3::4])
        else: 
            self.v_master.extend(u[0::100])
            self.ang_master.extend(u[1::100])
            self.v_slave.extend(u[2::100])
            self.ang_slave.extend(u[3::100])

        v_master = np.array(self.v_master)
        ang_master = np.array(self.ang_master)
        v_slave = np.array(self.v_slave)
        ang_slave = np.array(self.ang_slave)

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

        # deviation 
        x_master = [x[0] for x in past_traj_master[:]]
        y_master = [y[1] for y in past_traj_master[:]]
        x_slave = [x[0] for x in past_traj_slave[:]]
        y_slave = [y[1] for y in past_traj_slave[:]]
        if self.two_seconds: 
            x_master.extend([x for x in traj_master[:,0]])
            y_master.extend([y for y in traj_master[:,1]])
        if self.solver_param.base.enable_slave and self.plot_config['plot_slave']:
            if self.two_seconds: 
                x_slave.extend([x for x in traj_slave[:,0]])
                y_slave.extend([y for y in traj_slave[:,1]])
            constraint_dev = [ ((x_master[i]-x_slave[i])**2 + (y_master[i]-y_slave[i])**2)**0.5 for i in range(len(x_master))]

        u_time = np.array([t*self.solver_param.base.ts/5 for t in range(0,len(self.v_master))])

        try: 
            self.plot_queues['master_lin_vel'].put_nowait([ u_time, v_master])
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
            if self.plot_config['plot_slave'] and self.state == States.FORMATION:
                if self.two_seconds: 
                    self.plot_queues['constraint'].put_nowait([u_time, np.array(constraint_dev)])
                else: 
                    self.plot_queues['constraint'].put_nowait([u_time[1:], np.array(constraint_dev)])
        except Full:
            pass

        # Plot costs 
        all_positions_master = trajectory[:,0:2].T
        all_x_master = all_positions_master[0,:]
        all_y_master = all_positions_master[1,:]
        all_th_master = trajectory[:,2].T
        all_positions_slave = trajectory[:,3:5].T
        all_x_slave = all_positions_slave[0,:]
        all_y_slave = all_positions_slave[1,:]
        all_th_slave = trajectory[:,5].T
        self.mpc_generator.ref_points_master_x = np.array(self.mpc_generator.ref_points_master_x)
        self.mpc_generator.ref_points_master_y = np.array(self.mpc_generator.ref_points_master_y)
        self.mpc_generator.ref_points_master_th = np.array(self.mpc_generator.ref_points_master_th)
        self.mpc_generator.ref_points_slave_x = np.array(self.mpc_generator.ref_points_slave_x)
        self.mpc_generator.ref_points_slave_y = np.array(self.mpc_generator.ref_points_slave_y)
        self.mpc_generator.ref_points_slave_th = np.array(self.mpc_generator.ref_points_slave_th)
            
        master_ref_point_cost = self.mpc_generator.cost_dist2ref_points(all_x_master, all_y_master, all_th_master, self.mpc_generator.ref_points_master_x, self.mpc_generator.ref_points_master_y,  self.mpc_generator.ref_points_master_th, self.mpc_generator.q_pos_master, self.mpc_generator.q_theta_master)
        slave_ref_point_cost = self.mpc_generator.cost_dist2ref_points( all_x_slave,  all_y_slave,  all_th_slave,  self.mpc_generator.ref_points_slave_x,  self.mpc_generator.ref_points_slave_y,   self.mpc_generator.ref_points_slave_th,  self.mpc_generator.q_pos_slave,  self.mpc_generator.q_theta_slave)

        distance_cost = self.mpc_generator.cost_distance_between_atrs(all_x_master, all_y_master, all_x_slave, all_y_slave, self.mpc_generator.q_distance)
        distance_cost += self.mpc_generator.cost_distance_atr_soft_c( all_x_master, all_y_master, all_x_slave, all_y_slave, self.mpc_generator.q_distance_c)
        self.costs['cost_constraint'].append(distance_cost)
        self.costs_future['cost_future_constraint'] = np.sum((self.mpc_generator.cost_distance_between_atrs(all_x_master, all_y_master, all_x_slave, all_y_slave, self.mpc_generator.q_distance, individual_costs=True) , self.mpc_generator.cost_distance_atr_soft_c( all_x_master, all_y_master, all_x_slave, all_y_slave, self.mpc_generator.q_distance_c, individual_costs=True)), axis=0)
        
        self.costs['cost_master_line_deviation'].append(float(self.mpc_generator.cost_dist2ref_line(all_x_master, all_y_master, all_th_master, self.mpc_generator.ref_points_master_x, self.mpc_generator.ref_points_master_y, self.mpc_generator.q_cte, self.mpc_generator.q_line_theta)))
        self.costs_future['cost_future_master_line_deviation'] = self.mpc_generator.cost_dist2ref_line(all_x_master, all_y_master, all_th_master, self.mpc_generator.ref_points_master_x, self.mpc_generator.ref_points_master_y, self.mpc_generator.q_cte, self.mpc_generator.q_line_theta, individual_costs=True)
        # self.costs['cost_slave_line_deviation'].append(float(self.mpc_generator.cost_dist2ref_line(all_x_slave,all_y_slave, all_th_slave, self.mpc_generator.ref_points_slave_x, self.mpc_generator.ref_points_slave_y, self.mpc_generator.q_cte*self.mpc_generator.enable_distance_constraint,self.mpc_generator.q_line_theta*self.mpc_generator.enable_distance_constraint*0 )))

        self.costs['cost_master_dynamic_obs'].append(float(self.mpc_generator.cost_inside_dyn_ellipse2(all_x_master, all_y_master, self.mpc_generator.q_dyn_obs_c)))
        self.costs_future['cost_future_master_dynamic_obs'] = self.mpc_generator.cost_inside_dyn_ellipse2(all_x_master, all_y_master, self.mpc_generator.q_dyn_obs_c, individual_costs=True)
        self.costs['cost_slave_dynamic_obs'].append(float(self.mpc_generator.cost_inside_dyn_ellipse2(all_x_slave, all_y_slave, self.mpc_generator.q_dyn_obs_c)))
        self.costs_future['cost_future_slave_dynamic_obs'] = self.mpc_generator.cost_inside_dyn_ellipse2(all_x_slave, all_y_slave, self.mpc_generator.q_dyn_obs_c, individual_costs=True)

        cost_static_master = float(self.mpc_generator.cost_inside_static_object(all_x_master, all_y_master, self.mpc_generator.q_obs_c))
        cost_static_slave = float(self.mpc_generator.cost_inside_static_object(all_x_slave, all_y_slave, self.mpc_generator.q_obs_c))
        cost_bounds_master = float(self.mpc_generator.cost_outside_bounds(all_x_master, all_y_master, self.mpc_generator.q_obs_c))
        cost_bounds_slave = float(self.mpc_generator.cost_outside_bounds(all_x_slave, all_y_slave, self.mpc_generator.q_obs_c))
        self.costs['cost_master_static_obs'].append(float(cost_static_master + cost_bounds_master)) #TODO: Split boudns cost into seperate plot function
        self.costs_future['cost_future_master_static_obs'] = self.mpc_generator.cost_inside_static_object(all_x_master, all_y_master, self.mpc_generator.q_obs_c, individual_costs=True)
        self.costs['cost_slave_static_obs'].append(float(cost_static_slave + cost_bounds_slave)) #TODO: Split boudns cost into seperate plot function
        self.costs_future['cost_future_slave_static_obs'] = self.mpc_generator.cost_inside_static_object(all_x_slave, all_y_slave, self.mpc_generator.q_obs_c, individual_costs=True)
        #######  
        # Master control signal costs
        master_u = trajectory[:, [6, 7]].reshape(-1, 1)
        master_lin_vel_cost, master_ang_vel_cost = self.mpc_generator.cost_control(master_u, self.mpc_generator.q_lin_v, self.mpc_generator.q_ang)
        self.costs['cost_master_lin_vel'].append(float(master_lin_vel_cost))
        self.costs_future['cost_future_master_lin_vel'] = self.mpc_generator.cost_control(master_u, self.mpc_generator.q_lin_v, self.mpc_generator.q_ang, individual_costs=True)[0]
        self.costs['cost_master_ang_vel'].append(float(master_ang_vel_cost))
        self.costs_future['cost_future_master_ang_vel'] = self.mpc_generator.cost_control(master_u, self.mpc_generator.q_lin_v, self.mpc_generator.q_ang, individual_costs=True)[1]

        past_u = [[],[]]
        past_u[0] = u_master_prev
        past_u[1] = u_slave_prev
        #Master accelerations cost
        master_lin_vel_init = 0 if len(past_u[0]) < 4 else past_u[0][-4]
        master_ang_vel_init = 0 if len(past_u[0]) < 4 else past_u[0][-3]
        master_lin_acc, master_ang_acc = self.mpc_generator.calc_accelerations(master_u, master_lin_vel_init, master_ang_vel_init)

        master_lin_acc_cost = self.mpc_generator.cost_linear_acc(master_lin_acc, self.mpc_generator.q_lin_acc, self.mpc_generator.q_lin_acc)        
        # master_lin_acc_cost += self.mpc_generator.cost_acc_constraint(master_lin_acc, self.solver_param.base.lin_acc_min, self.solver_param.base.lin_acc_max, self.mpc_generator.q_acc_c)
        self.costs['cost_master_lin_acc'].append(float(master_lin_acc_cost))
        self.costs_future['cost_future_master_lin_acc'] = self.mpc_generator.cost_linear_acc(master_lin_acc, self.mpc_generator.q_lin_acc, self.mpc_generator.q_lin_acc, individual_costs=True)    

        master_ang_acc_cost = self.mpc_generator.cost_angular_acc(master_ang_acc, self.mpc_generator.q_ang_acc)
        # master_ang_acc_cost += self.mpc_generator.cost_acc_constraint(master_ang_acc, self.solver_param.base.lin_acc_min, self.solver_param.base.lin_acc_max, self.mpc_generator.q_acc_c)
        self.costs['cost_master_ang_acc'].append(float(master_ang_acc_cost))
        self.costs_future['cost_future_master_ang_acc'] = self.mpc_generator.cost_angular_acc(master_ang_acc, self.mpc_generator.q_ang_acc, individual_costs=True)

        # Master line deviation cost
        # slave control signal costs
        slave_u = trajectory[:, [8, 9]].reshape(-1, 1)
        slave_lin_vel_cost, slave_ang_vel_cost = self.mpc_generator.cost_control(slave_u, self.mpc_generator.q_lin_v, self.mpc_generator.q_ang)
        self.costs['cost_slave_lin_vel'].append(float(slave_lin_vel_cost))
        self.costs_future['cost_future_slave_lin_vel'] = self.mpc_generator.cost_control(slave_u, self.mpc_generator.q_lin_v, self.mpc_generator.q_ang, individual_costs=True)[0]
        self.costs['cost_slave_ang_vel'].append(float(slave_ang_vel_cost))
        self.costs_future['cost_future_slave_ang_vel'] = self.mpc_generator.cost_control(slave_u, self.mpc_generator.q_lin_v, self.mpc_generator.q_ang, individual_costs=True)[1]

        # Slave accelerations cost
        slave_lin_vel_init = 0 if len(past_u[1]) < 4 else past_u[1][-4]
        slave_ang_vel_init = 0 if len(past_u[1]) < 4 else past_u[1][-3]
        slave_lin_acc, slave_ang_acc = self.mpc_generator.calc_accelerations(slave_u, slave_lin_vel_init, slave_ang_vel_init)

        slave_lin_acc_cost = self.mpc_generator.cost_linear_acc(slave_lin_acc, self.mpc_generator.q_lin_acc, self.mpc_generator.q_lin_ret)
        self.costs['cost_slave_lin_acc'].append(float(slave_lin_acc_cost))
        self.costs_future['cost_future_slave_lin_acc'] = self.mpc_generator.cost_linear_acc(slave_lin_acc, self.mpc_generator.q_lin_acc, self.mpc_generator.q_lin_ret, individual_costs=True) 

        slave_ang_acc_cost = self.mpc_generator.cost_angular_acc(slave_ang_acc, self.mpc_generator.q_ang_acc)
        self.costs['cost_slave_ang_acc'].append(float(slave_ang_acc_cost))
        self.costs_future['cost_future_slave_ang_acc'] = self.mpc_generator.cost_angular_acc(slave_ang_acc, self.mpc_generator.q_ang_acc, individual_costs=True)

        # Slave line deviation cost & Constraints cost & Vel ref costs
        vel_ref_cost = float(self.mpc_generator.cost_v_ref_difference(master_u.flatten(), np.array(self.mpc_generator.v_ref_master), self.mpc_generator.q_d_lin_vel_upper, self.mpc_generator.q_d_lin_vel_lower))
        vel_ref_cost += float(self.mpc_generator.cost_ang_vel_ref_difference(master_u.flatten(), np.array(self.mpc_generator.ang_vel_ref_master), self.mpc_generator.q_d_ang_vel))
        self.costs['cost_master_vel_ref'].append(vel_ref_cost)
        self.costs_future['cost_future_master_vel_ref'] = self.mpc_generator.cost_v_ref_difference(master_u.flatten(), np.array(self.mpc_generator.v_ref_master), self.mpc_generator.q_d_lin_vel_upper, self.mpc_generator.q_d_lin_vel_lower, individual_costs=True) + self.mpc_generator.cost_ang_vel_ref_difference(master_u.flatten(), np.array(self.mpc_generator.ang_vel_ref_master), self.mpc_generator.q_d_ang_vel, individual_costs=True) 
        self.costs['cost_slave_vel_ref'].append(vel_ref_cost)
        
        past_time = np.linspace(0, self.solver_param.base.ts*(len(self.costs['cost_master_line_deviation'])) - self.solver_param.base.ts, len(self.costs['cost_master_line_deviation']))
        future_time = np.linspace(0, self.solver_param.base.ts*(len(self.costs_future['cost_future_master_line_deviation'])) - self.solver_param.base.ts, len(self.costs_future['cost_future_master_line_deviation'])) + past_time[-1]

        try: # Try statement needed in case the queue is full
            for key in self.costs:
                try:
                    if self.costs[key] != []:
                        self.plot_queues[key].put_nowait([past_time, self.costs[key]])
                except KeyError:
                    print(f"Key not found:  {key}")
        except Full:
            pass

        try: # Try statement needed in case the queue is full
            for key in self.costs_future:
                try:
                    if get_length(self.costs_future[key]) > 0:
                        future_costs = [float(self.costs_future[key][i]) for i in range(get_length(self.costs_future[key]))]
                        self.plot_queues[key].put_nowait([future_time, future_costs])
                except KeyError:
                    print(f"Key not found:  {key}")
        except Full:
            pass

        self.times['plot'].append(perf_counter_ns() - start_time)

    def kill(self):
        if self.verbose:
            print(f"{self.print_name} is killing itself.")
        self.plot_process.kill()
        
        