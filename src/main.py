from ctypes import POINTER
import os, sys, json
# import pickle
from pathlib import Path
from typing import Counter

import numpy as np

from shapely.geometry import LineString
from shapely.geometry import box as Box
from shapely.geometry import Polygon
import matplotlib.pyplot as plt
# To suppress the warnings about elementwise comparison...
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

def init(build=False, self_destruct=False):
    # Get path to this file
    file_path = Path(__file__)

    # For simulation. copy obstacles to obstacels_copy since obstacles_copy changEs for the dynamic simulation
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


def init_obs():
    # Get path to this file and load obstacle definitions
    file_path = Path(__file__)
    obs_original = os.path.join(str(file_path.parent.parent), 'data', 'obstacles.json')
    with open(obs_original) as f:
        obs_json = json.load(f)

    # static obstacle dict was commented below for a better visualization, if needed, the coodnites for static obstacle should be added back in the 'obstacles.json' file under 'data' folder, changes also are needed on line 92-94 in "mpc_generator" file under 'mpc' folder, line 592-593 in file 'trajectory_generator', line 46-48 in file 'trajectory_generator', , line 122-123 in file 'trajectory_generator'
    # # Construct obstacles as objects in shapely.geometry for collision detection
    # static_obs = obs_json['static']
    # static_dict = dict()
    # index = 0
    # for obs in static_obs:
    #     obs_shapely = Polygon(obs)
    #     # Compute axis-aligned bounding boxes
    #     bounds = obs_shapely.bounds
    #     box = Box(bounds[0], bounds[1], bounds[2], bounds[3])
    #     # Store
    #     static_dict[str(index)] = [box, static_obs]
    #     index += 1
    # # Save
    # # shapely_pickle = os.path.join(str(file_path.parent.parent), 'data', 'shapely_obstacles_w_bounding_boxes.p')
    # # with open(shapely_pickle, mode='wb') as f:
    # #     pickle.dump(static_dict,f)
    # return static_dict

# Checks if there is a collision with the obstacles (only for static obstacles for now)
# TODO: Implement a function to check wether or not a trajectory has collided with any obstacle.
def collisionDetection(trajList):
    # trajList should be a list of generated trajectores:
    # [[(x0_m,y0_m,theta0_m),...,(xN_m,yN_m,thetaN_m)],[(x0_s,y0_s,theta0_s),...,(xN_s,yN_s,thetaN_s)]], for N number of points.
    # obstacle_corners could for example be a list containing the coordinates of all PADDED obstacles:
    # [[o1_x,o1_y],...,[oM_x,oM_y]], for M obstacles.
    # TODO: Fix the padded obstacles coordinates!

    obstacle = Polygon([(5.0, 4.0), (5.0, 5.0), (6.0, 5.0), (6.0, 4.0)])
    nr_of_points = len(trajList[0])
    master_trajectory = trajList[0]
    slave_trajectory = trajList[1]
    counter = 0
    collided_list = []  # A list to save all trajectory points that collided
    a = 1
    for i in range(nr_of_points):
        # Cargo defined as a line
        # cargo = LineString([master_trajectory[i][:2],slave_trajectory[i][:2]])
        x_master = master_trajectory[i][0]
        y_master = master_trajectory[i][1]
        x_slave = slave_trajectory[i][0]
        y_slave = slave_trajectory[i][1]

        master_corner_1 = (x_master, y_master) + a*(y_slave-y_master, x_master-x_slave)/np.linalg.norm([y_master-y_slave, x_master-x_slave])
        master_corner_2 = (x_master, y_master) - a*(y_slave-y_master, x_master-x_slave)/np.linalg.norm([y_master-y_slave, x_master-x_slave])
        slave_corner_1 = (x_slave, y_slave)    + a*(y_slave-y_master, x_master-x_slave)/np.linalg.norm([y_master-y_slave, x_master-x_slave])
        slave_corner_2 = (x_slave, y_slave)    - a*(y_slave-y_master, x_master-x_slave)/np.linalg.norm([y_master-y_slave, x_master-x_slave])

        # Cargo defined as a polygon
        cargo = Polygon([master_corner_1,master_corner_2,slave_corner_2,slave_corner_1])

        if cargo.intersects(obstacle):
            counter = counter + 1
            collision_intersection = cargo.intersection(obstacle)

            # Plots all senarios of detected collisions.
            plot_polygons_w_atr([obstacle,cargo], [x_master, y_master], [x_slave, y_slave])
            
            intersection_area = collision_intersection.area
            
            #Contains the coordinates where the trajectory collided
            collided_list.append([master_trajectory[i][:2],slave_trajectory[i][:2]])
            print("Oh no collision detected!")

    
    print("Collided: ", counter)
    return collided_list



# A function to plot several polygons.
def plot_polygons_w_atr(polygons, master_pos, slave_pos):
    # polygons is a list of Polygon objects
    for p in polygons:
        x,y = p.exterior.xy
        plt.plot(x,y)
    plt.plot(master_pos[0],master_pos[1], 'gx')
    plt.plot(slave_pos[0],slave_pos[1], 'rx')
    plt.show()


def main(build=True, destroy_plots=False):
    traj_gen = init(build=build, self_destruct=destroy_plots)
    try:
        traj_gen.run(plot=1)
    finally:
        print("Should kill all running processes")
    generated_trajectory = traj_gen.getGeneratedTrajectory()

    # out = collisionDetection(generated_trajectory)

if __name__ == '__main__':
    
    main()
    print("All done.\nGood bye")
