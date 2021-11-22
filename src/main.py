import os, sys, json
from pathlib import Path

from trajectory_generator import TrajectoryGenerator
from path_planner.global_path_planner import GlobalPathPlanner


def init(build=False, self_destruct=False):

        # Get path to this file
        file_path = Path(__file__)

        # For simulation. copy obstacles to obstacels_copy since obstacles_copy changs for the dynamic simulation
        obs_original = os.path.join(str(file_path.parent.parent), 'data', 'obstacles.json')
        obs_copy     = os.path.join(str(file_path.parent.parent), 'data', 'obstacles_copy.json')
        with open(obs_original) as f: 
            obs_json = json.load(f)
        with open(obs_copy, mode='w') as f: 
            json.dump(obs_json,f)
        
        print("test json ", obs_json)

        # Given start and goal poses of the robots
        start_master = (4.5, 1, 1.57)
        start_slave = (4.5,0,1.57)

        #ORIGINAL NODES
        master_goal = [(5, 2, 1.57), (5, 9, 1.57), (4, 10, 1.57)] # for different modes
        slave_goal = [(5, 1, 1.57), (5, 8, 1.57), (4, 9, 1.57)]

        #NEW NODES
        # master_goal = [(5, 2, 1.57), (8, 6, 1.57), (10, 8, 1.57)] # for different modes
        # slave_goal = [(5, 1, 1.57), (7, 6, 1.57), (9, 8, 1.57)]
    
        # Gloabal path
        map_fp = os.path.join(str(file_path.parent.parent), 'data', 'map.json')
        print("map fp", map_fp)
        gpp = GlobalPathPlanner(map_fp)
        global_path = gpp.search_astar(master_goal[0][:2], master_goal[1][:2]) # Send the start and end position for the master to A* alg.
        print('global path ', global_path)

        # Create the trajectory generator and set global path
        start_pose = [start_master, start_slave]
        traj_gen = TrajectoryGenerator(start_pose, verbose=True, name="TrajGen", self_destruct=self_destruct, master_goal= master_goal, slave_goal=slave_goal, build_solver=build)
        traj_gen.set_global_path(global_path)
        return traj_gen


# Checks if there is a collision with the obstacles (only for static obstacles for now)
# TODO: Implement a function to check wether or not a trajectory has collided with any obstacle.
def collisionDetection(trajList, obstacle_corners):
    # trajList should be a list of generated trajectores: [[(x0_m,y0_m,theta0_m),...,(xN_m,yN_m,thetaN_m)],[(x0_s,y0_s,theta0_s),...,(xN_s,yN_s,thetaN_s)]], for N number of points.
    # obstacle_corners could for example be a list containing the coordinates of all PADDED obstacles: [[o1_x,o1_y],...,[oM_x,oM_y]], for M obstacles.
    # TODO: Fix the padded obstacles coordinates!

    #For every pair of nodes compute the line between them
    nr_of_points = len(trajList[0])
    master_trajectory = trajList[0]
    slave_trajectory = trajList[1]
    for i in enumerate(nr_of_points):
        x_m = master_trajectory[i][0]
        y_m = master_trajectory[i][1]
        x_s = slave_trajectory[i][0]
        y_s = slave_trajectory[i][1]

        #Line equation to find the line between the two ATRs, defined on the closed interval [x_s,x_m], [y_s,y_m]
        k = (y_m - y_s)/(x_m - x_s)
        m = y_m - k*x_m

        # Loop through all the segments that encapules the obstacle and see if they intersect with any point on the line that represents the load.

def main(build=False, destroy_plots=False):
    traj_gen = init(build=build, self_destruct=destroy_plots)
    try:
        traj_gen.run(plot=1)
    finally:
        print("Should kill all running processes")
    generated_trajectory = traj_gen.getGeneratedTrajectory()
    # print("Generated trajectory ", generated_trajectory)
        

if __name__ == '__main__':
    
    main()
    print("All done.\nGood bye")
