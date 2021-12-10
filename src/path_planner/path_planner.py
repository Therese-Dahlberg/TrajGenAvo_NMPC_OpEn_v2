import os
import json
from pathlib import Path
from time import perf_counter_ns

import numpy as np

from shapely import geometry
from extremitypathfinder.extremitypathfinder import PolygonEnvironment

class PathPlanner:
    """Class which does local path planning. Local path finding utilizes visibility graph to 
    find the shortest path around unexpected obstacles.
    """

    def __init__(self, config, plot_config, plot_queues):
        self.config = config
        self.plot_config = plot_config
        self.plot_queues = plot_queues
        self.traversed_planned_path = [] # ONLY FOR SIMULATION
        self.path = None
        self.path_shapely = None
        self.times = {'global':[], 'local':[], 'unexpected': [], 'plot':[]}


    def set_global_path(self, path): 
        self.path = path
        self.path_shapely = [geometry.Point(p) for p in path]
        

    def visability_graph(self, boundary, obstacles, start_pos, end_pos):
        self.env = PolygonEnvironment()
        # Give obstacles and boundaries to environment
        self.env.store(boundary, obstacles)
        # Prepare the visibility graph 
        self.env.prepare()

        path, distance = self.env.find_shortest_path(start_pos, end_pos)
        # If path is empty then there is no path
        if path == []:
            raise Exception("No path to be found")

        return path

    def local_path_plan(self, x_cur, unexpected_obstacles, unexpected_obstacles_shapely, static_obstacles, boundary):
        """This method plans a path around unexpected obstacles using visability graphs.
        The update path is stored in 

        Args:
            
            x_cur ([list or tuple]): [x, y], (x, y)

        Returns:
            True/False if the path is updated 
        """
        start_time = perf_counter_ns()

        collision = self.path_through_obs(unexpected_obstacles_shapely)
        if not collision:
            return False

        start_pos_idx, end_pos_idx = self.find_start_end_pos_around_obs(unexpected_obstacles_shapely)

        all_obs = [[tuple(pos) for pos in ob] for ob in unexpected_obstacles] + [[tuple(pos) for pos in ob] for ob in static_obstacles]
        
        for i in range(len(start_pos_idx)):
            start = self.path[start_pos_idx[i]]
            end = self.path[end_pos_idx[i]]
            path = self.visability_graph(boundary, all_obs, start, end)
            self.path[start_pos_idx[i] : end_pos_idx[i]+1] = path

        # Update the shapely path
        self.path_shapely = [geometry.Point(p) for p in self.path]

        # Save the traversed portion of the path. Only for simulation
        self.save_traversed_planned_path(x_cur)

        self.times['local'].append(perf_counter_ns() - start_time)
        return True

    def save_traversed_planned_path(self, x_cur):
        """ Save the traversed portion of the path. Only for simulation
        """
        x_cur_node_idx = np.argmin(np.linalg.norm(np.array(self.path)-np.array(x_cur[:2]), ord=2, axis=1))

        self.traversed_planned_path.extend(self.path[:x_cur_node_idx+1])
    

    def find_start_end_pos_around_obs(self, unexpected_obstacles):
        """Calculates the start and end nodes around obstacles intersecting the path.

        Args:
            unexpected_obstacles (list of lists of tuples): a list that contains a list of obstacles. Each obstacle is a polygon defined by tuples containing it's vertices.

        Returns:
            idx_of_start_nodes [list]: nodes at which to start to deviate from the original path to avoid obstacles
            idx_of_end_nodes [list]: nodes at which to end the deviation from the original path to avoid obstacles
        """
        
        obstacles = unexpected_obstacles

        # Find the obs which intersect the path
        idx_of_intersecting_obs = []
        for i, ob in enumerate(obstacles):
            for node in self.path_shapely:
                if not ob.intersection(node).is_empty:
                    idx_of_intersecting_obs.append(i)
                    break

        # Go through the path and find unavailable nodes
        unavailable_nodes = []
        for i, node in enumerate(self.path_shapely):
            for j in idx_of_intersecting_obs:
                ob = obstacles[j]
                if ob.exterior.distance(node) <= self.config.distance_to_avoid_unexpected_obstacles and not node == self.path_shapely[-1]:
                    unavailable_nodes.append(i)
                elif not ob.intersection(node).is_empty:
                    unavailable_nodes.append(i)

        # Find start and end nodes
        main_list = np.setdiff1d([x for x in range(len(self.path))], unavailable_nodes, assume_unique=True)
        idx_of_start_nodes = [main_list[i] for i in range(len(main_list)-1) if not main_list[i]+1==main_list[i+1]]
        idx_of_end_nodes = [main_list[i] for i in range(1, len(main_list)) if not main_list[i-1] == main_list[i]-1]

        return idx_of_start_nodes, idx_of_end_nodes

    def path_through_obs(self, unexpected_obstacles):
        """Find if the path goes through any unexpected obstacle

        Args:
            unexpected_obstacles ([type]): [description]

        Returns:
            Boolean: True if blocked, None if not
        """

        for ob in unexpected_obstacles:
            x, y = ob.exterior.xy
            for node in self.path_shapely:
                if not ob.intersection(node).is_empty:
                # If the point is on the boundary it's ok
                    if (node.bounds[0] in x) and (node.bounds[1] in y):
                        continue
                    return True

    def plot(self):
        start_time = perf_counter_ns()
        ################################### Temporary solution for plotting
        file_path = Path(__file__)
        map_fp = os.path.join(str(file_path.parent.parent.parent), 'data', 'map.json')
        map_json = json.load(open(map_fp))
        x = []
        y = []
        for node in map_json['nodes']: 
            x.append(node['coord'][0])
            y.append(node['coord'][1])
        data = [x,y]
        ##################################
        try: self.plot_queues['nodes'].put_nowait(data)
        except: pass

        # Plot Planned path
        data = [[x for x, y in self.path], [y for x, y in self.path]]
        try: self.plot_queues['planned_path'].put_nowait(data)
        except: pass

        self.times['plot'].append(perf_counter_ns() - start_time)



    