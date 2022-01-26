import os, json
from pathlib import Path

# To suppress the warnings about elementwise comparison...
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

def init(build=False, self_destruct=False):
    # Get path to this file
    file_path = Path(__file__)

    # For simulation. copy obstacles to obstacles_copy since obstacles_copy changes for the dynamic simulation
    obs_original = os.path.join(str(file_path.parent.parent), 'data', 'obstacles.json')
    obs_copy     = os.path.join(str(file_path.parent.parent), 'data', 'obstacles_copy.json')
    with open(obs_original) as f:
        obs_json = json.load(f)
    with open(obs_copy, mode='w') as f:
        json.dump(obs_json,f)

    print("test json ", obs_json)

    # Given start and goal poses of the robots
    start_master = (4.5, 1, 1.57)
    start_slave = (4.5, 0, 1.57)

    # ORIGINAL NODES
    master_goal = [(5, 2, 1.57), (5, 9, 1.57), (4, 10, 1.57)] # for different modes
    slave_goal = [(5, 1, 1.57), (5, 8, 1.57), (4, 9, 1.57)]

    # NEW NODES
    # master_goal = [(5, 2, 1.57), (8, 6, 1.57), (10, 8, 1.57)] # for different modes
    # slave_goal = [(5, 1, 1.57), (7, 6, 1.57), (9, 8, 1.57)]

    # Global path
    map_fp = os.path.join(str(file_path.parent.parent), 'data', 'map.json')
    print("map fp", map_fp)
    from trajectory_generator import TrajectoryGenerator
    from path_planner.global_path_planner import GlobalPathPlanner
    gpp = GlobalPathPlanner(map_fp)
    global_path = gpp.search_astar(master_goal[0][:2], master_goal[1][:2]) # Send the start and end position for the master to A* alg.
    print('global path ', global_path)

    # Create the trajectory generator and set global path
    start_pose = [start_master, start_slave]
    traj_gen = TrajectoryGenerator(start_pose, verbose=True, name="TrajGen",
                                    self_destruct=self_destruct, master_goal= master_goal,
                                    slave_goal=slave_goal, build_solver=build)
    traj_gen.set_global_path(global_path)
    return traj_gen

def main(build=True, destroy_plots=False):
    traj_gen = init(build=build, self_destruct=destroy_plots)
    try:
        traj_gen.run(plot=1)
    finally:
        print("Should kill all running processes")
    generated_trajectory = traj_gen.getGeneratedTrajectory()


if __name__ == '__main__':
    
    main()
    print("All done.\nGood bye")
