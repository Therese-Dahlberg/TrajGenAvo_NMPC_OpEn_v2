import yaml

""" 
    File that contains all the neccessary configuration parameters for the 
    MPC Trajectory Generation Module
"""
        
required_config_params = {
    'n_hor': 'The length of the receding horizon controller', 
    'lin_vel_min': 'Vehicle contraint on the minimal velocity possible', 
    'lin_vel_max': 'Vehicle contraint on the maximal velocity possible', 
    'lin_acc_min': 'Vehicle contraint on the maximal linear retardation', 
    'lin_acc_max': 'Vehicle contraint on the maximal linear acceleration', 
    'lin_jerk_min': 'Max jerk', 
    'lin_jerk_max': 'Min jerk', 
    'ang_vel_max': 'Vehicle contraint on the maximal angular velocity', 
    'ang_acc_max': 'Vehicle contraint on the maximal angular acceleration (considered to be symmetric)', 
    'ang_jerk_min': "Min jerk", 
    'ang_jerk_max': "Max jerk", 
    'throttle_ratio': 'What percent of the maximal velocity should we try to', 
    'num_steps_taken': 'How many steps should be taken from each mpc-solution. Range (1 - n_hor)', 
    'ts': 'Size of the time-step', 
    'vel_red_steps' :'Steps for velocity reduction', 
    'nx': 'Number of states for the robot (x, y, theta)', 
    'nu': 'Number of control inputs', 
    'vehicle_width': 'Vehicle width in meters', 
    'vehicle_margin': 'Extra margin used for padding in meters', 
    'build_type': "Can have 'debug' or 'release'", 
    'build_directory': 'Name of the directory where the build is created', 
    'bad_exit_codes': 'Optimizer specific names', 
    'optimizer_name': 'Optimizer type, default "navigation"', 
    'constraint' : 'Dict with dual ATR constraints', 
    'distance_to_avoid_unexpected_obstacles': 'How far ahead of unexpected obstacles it should start to avoid', 
    'enable_slave': 'If there should should be a slave. If set to False there will only be a master robot.', 
    'aggressive_factor' :    ' Between 0 & 1, between old people driving & rally driver. ',
    'trajectory_length': 'How long the trajectory should be', 
    'dist_to_start_turn': 'How far ahead of the turns the master and slave should prepare for turning', 
    'ang_detect_dyn_obs': "The angle around the master at which to detect dynamic obstacles. It's symmetric so pi would yield a full circle",
    'dist_detect_dyn_obs': "The radius of the circle sector in which dynamic obstacles are detected.",
    'min_vertices':  'The minimum amount of vertices per obstacle', 
    'max_vertices': 'The maximum amount of vertices per obstacle', 
    'n_param_line': 'Number of variables per line',
    'n_param_vertex': 'Number of variables per vertex',
    'n_param_dyn_obs': 'Number of variables per dynamic obstacle', 
    'n_obs_of_each_vertices':  'How many obstacles there should be of each kind of obstacle', 
    'n_dyn_obs':  'How many dynamic obstacles there should be', 
    'n_bounds_vertices':  'Number of vertices for drivable area', 
    'n_vert_cargo': 'Number of vertices of the cargo',
}

required_weights_params = {
    'n_weights' : 'Number of weight parameters', 
    'q_lin_v': 'Cost for linear velocity control action', 
    'q_lin_acc': 'Cost for linear acceleration', 
    'q_lin_ret': 'Cost for linear retardation', 
    'q_lin_jerk': 'Cost for linear jerk', 
    'q_ang': 'Cost angular velocity control action', 
    'q_ang_acc': 'Cost angular acceleration', 
    'q_ang_jerk': 'Cost for angular jerk', 
    'q_cte': 'Cost for cross-track-error from each line segment', 
    'q_pos_master': 'Cost for position deviation (each time step vs reference point)',
    'q_pos_slave':  'Cost for position deviation (each time step vs reference point)',
    'q_d_lin_vel_upper': 'Cost for upper speed deviation each time step',
    'q_d_lin_vel_lower': 'Cost for lower speed deviation each time step',
    'q_reverse' : ' Cost for reverse ',
    'q_d_ang_vel': 'Cost for speed deviation each time step',
    'q_theta_master': 'Cost for each heading relative to the refernce position',
    'q_theta_slave': 'Cost for each heading relative to the refernce position',
    'q_pos_N': 'Terminal cost; error relative to final reference position', 
    'q_theta_N': 'Terminal cost; error relative to final reference heading', 
    'q_distance': 'Cost for deviating from distance', 
    'q_theta_diff': 'Cost for angular difference between robots', 
    'q_distance_c' : 'Cost for deviating from d + tolerance, soft constraint', 
    'q_obs_c' : 'Cost for being inside an object, soft constraint', 
    'q_dyn_obs_c' : 'Cost for being inside an dynamic object, soft constraint', 
    'q_future_dyn_obs' : 'Cost for being inside a future dynamic object', 
    'q_acc_c' : 'Cost for acceleration, soft constraint', 
    'enable_distance_constraint' : 'Cost for cross-track-error factor', 
    'q_formation_ang' : 'Cost for deviating from checkpoints', 
    'q_line_theta': 'Cost for deviating of the reference angle which is based on the lines',
}

class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__

class Configurator:
    def __init__(self, yaml_fp):
        self.print_name = "[CONFIGURATOR]"
        self.fp = yaml_fp
        print(f"{self.print_name}  Loading configuration from '{self.fp}'")
        with open(self.fp, 'r') as stream:
            self.input = yaml.safe_load(stream)
        self.args = dotdict()

    def configurate_key(self, key):
        value = self.input.get(key)
        if value is None:
            print(f"{self.print_name}  Can not find '{key}' in the YAML-file. Explanation is: '{required_config_params[key]}'")
            raise RuntimeError(f'{self.print_name}  Configuration is not properly set')
        self.args[key] = value

    def configurate(self):
        print(f'{self.print_name}  STARTING CONFIGURATION...')
        for key in required_config_params:
            self.configurate_key(key)
        print(f'{self.print_name}  CONFIGURATION FINISHED SUCCESSFULLY...')
        return self.args

class Weights:
    def __init__(self, yaml_fp):
        self.print_name = "[WEIGHTS]"
        self.fp = yaml_fp
        print(f"{self.print_name} Loading weights from '{self.fp}'")
        with open(self.fp, 'r') as stream:
            self.input = yaml.safe_load(stream)
        self.args = dotdict()

    def configurate_key(self, key):
        value = self.input.get(key)
        if value is None:
            print(f"{self.print_name}  Can not find '{key}' in the YAML-file. Explanation is: '{required_weights_params[key]}'")
            raise RuntimeError(f'{self.print_name}  weights is not properly set')
        self.args[key] = value

    def configurate(self):
        print(f'{self.print_name}  STARTING WEIGHTS...')
        for key in required_weights_params:
            self.configurate_key(key)
        print(f'{self.print_name}  CONFIGURATION FINISHED SUCCESSFULLY...')
        return self.args


class SolverParams:
    def __init__(self, base, line_follow_weights, traj_follow_weights, pos_goal_weights, line_follow_weights_aggresive, traj_follow_weights_aggressive):
        self.base = base
        self.line_follow_weights = line_follow_weights
        self.line_follow_weights_aggressive = line_follow_weights_aggresive
        self.traj_follow_weights = traj_follow_weights
        self.traj_follow_weights_aggressive = traj_follow_weights_aggressive
        self.pos_goal_weights = pos_goal_weights