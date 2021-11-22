import os, sys, json
from pathlib import Path
from typing import Counter

from trajectory_generator import TrajectoryGenerator
from path_planner.global_path_planner import GlobalPathPlanner

from shapely.geometry import LineString
from shapely.geometry import Point
from shapely.geometry import Polygon
import matplotlib.pyplot as plt

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
def collisionDetection(trajList):
    # trajList should be a list of generated trajectores: [[(x0_m,y0_m,theta0_m),...,(xN_m,yN_m,thetaN_m)],[(x0_s,y0_s,theta0_s),...,(xN_s,yN_s,thetaN_s)]], for N number of points.
    # obstacle_corners could for example be a list containing the coordinates of all PADDED obstacles: [[o1_x,o1_y],...,[oM_x,oM_y]], for M obstacles.
    # TODO: Fix the padded obstacles coordinates!

    padded_obstacle = Polygon([(4.5,3.5),(4.5,5.5),(6.5,5.5),(6.5,3.5)])
    nr_of_points = len(trajList[0])
    master_trajectory = trajList[0]
    slave_trajectory = trajList[1]
    counter = 0
    collided_list = []  # A list to save all trajectory points that collided
    a = 0.01
    for i in range(nr_of_points):
        # Load defined as a line
        # load = LineString([master_trajectory[i][:2],slave_trajectory[i][:2]])

        master_corner_1 = (master_trajectory[i][0] + a, master_trajectory[i][1])
        master_corner_2 = (master_trajectory[i][0] - a, master_trajectory[i][1])
        slave_corner_1 = (slave_trajectory[i][0] + a, slave_trajectory[i][1])
        slave_corner_2 = (slave_trajectory[i][0] - a, slave_trajectory[i][1])

        # Load defined as a polygon
        load = Polygon([master_corner_1,master_corner_2,slave_corner_2,slave_corner_1])

        if load.intersects(padded_obstacle):
            counter = counter + 1
            collision_intersection = load.intersection(padded_obstacle)

            # Plots all senarios of detected collisions.
            plot_polygons([padded_obstacle,load])
            
            intersection_area = collision_intersection.area
            
            #Contains the coordinates where the trajectory collided
            collided_list.append([master_trajectory[i][:2],slave_trajectory[i][:2]])
            print("Oh no collision detected!")


    
    print("Collided: ", counter)
    return collided_list

# A function to plot several polygons.
def plot_polygons(polygons):
    # polygons is a list of Polygon objects
    for p in polygons:
        x,y = p.exterior.xy
        plt.plot(x,y)
    plt.show()


def main(build=False, destroy_plots=True):
    traj_gen = init(build=build, self_destruct=destroy_plots)
    try:
        traj_gen.run(plot=1)
    finally:
        print("Should kill all running processes")
    generated_trajectory = traj_gen.getGeneratedTrajectory()
    out = collisionDetection(generated_trajectory)
        

if __name__ == '__main__':
    
    main()
    print("All done.\nGood bye")
