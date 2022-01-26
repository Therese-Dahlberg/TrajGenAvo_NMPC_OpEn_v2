# Python imports
import os, sys, json
from pathlib import Path
from time import perf_counter_ns
from queue import Full

import numpy as np

import pyclipper
import shapely

from sympy import symbols, solve, lambdify
from sympy.testing.pytest import ignore_warnings

# Import CGAL, from own directory
from CGAL.CGAL_Kernel import Point_2
from CGAL.CGAL_Triangulation_2 import Constrained_Delaunay_triangulation_2

class DynamicObstacle:
    def __init__(self, polygon, p1, p2, vel, n_steps_ahead):
        """Creates a dynamic obstacle class

        Args:
            polygon (list of tuples ): The tuples describes the extremities of the polygon
            p1 (tuple): start position
            p2 (tuple): end position
            dp (tuple): (dx, dy)
        """
        self.polygon = np.array(polygon, dtype='float')
        self.center_pos = np.mean(self.polygon, axis=0)
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)
        self.vel = np.array(vel)

        self.future_polygons = []
        self.n_steps_ahead = n_steps_ahead

    def update(self):
        """Updates the current state to one step forward and updates future polygons with future polygons
        """

        def step(polygon, center_pos):
            polygon += self.vel
            center_pos = np.mean(polygon, axis=0)

            return polygon, center_pos

        for i in range(self.n_steps_ahead):
            self.polygon, self.center_pos = step(self.polygon, self.center_pos)
            self.future_polygons.append(self.polygon.copy())

class FaceInfo2(object):
    def __init__(self):
        self.nesting_level = -1

    def in_domain(self):
        return (self.nesting_level % 2) != 1

class ObstacleHandler:
    def __init__(self, config, plot_config, plot_queues):
        self.config = config
        self.plot_config = plot_config
        self.plot_queues = plot_queues

        self.inflator = pyclipper.PyclipperOffset()
        self.times = {'set_dynamic':[], 'set_static':[], 'set_unexpected':[], 'get_closest_obs':[], 'obstacle_as_triangle':[], 'obstacle_as_inequality': [], 'obstacle_as_ellipse':[], 'plot':[]}

        # Calculate the algebraic solution to the mvee equations which represent dynamic obstacles
        self.solve_mvee()
        self.count = 0

    def get_boundary(self): 
        map_fp = os.path.join(str(Path(__file__).parents[2]), 'data', 'map.json')
        with open(map_fp) as f: 
            map_json = json.load(f)
        extra_obs, new_bounds = self.bounds_to_obstacles_and_convex_hull(map_json['boundary'])
        boundary_original = np.array(new_bounds.copy())
        boundary_padded = self.pad_obstacles(np.array([boundary_original.tolist()]), decrease=True)[0]
        return boundary_padded

    def get_dynamic_obstacles(self): 
        obs_fp = os.path.join(str(Path(__file__).parent.parent.parent), 'data', 'obstacles_copy.json')
        with open(obs_fp) as f: 
            obs_json = json.load(f)
        
        dyn_obs_list = []
        for elem in obs_json['dynamic']: 
            dyn_obs_list.append(DynamicObstacle(elem['vertices'],elem['start'], elem['goal'], elem['vel'], elem['n_hor']))

        [ob.update() for ob in dyn_obs_list]
        dyn_obs = [ob.future_polygons for ob in dyn_obs_list] 

        # This should maybe only be done with the closest obstacles
        dynamic_original_obs = np.array(dyn_obs.copy())
        dynamic_padded_obs = self.pad_obstacles(dynamic_original_obs, dynamic=True)
        dynamic_obstacles_shapely = [self.obs_as_shapely_polygon(ob) for ob in dynamic_padded_obs]
        return dynamic_original_obs, dynamic_padded_obs, dynamic_obstacles_shapely

    def update_dynamic_obstacles(self):
        """Advances the dynamic obstacles 1 time step ahead
        """
       
        obs_fp = os.path.join(str(Path(__file__).parents[2]), 'data', 'obstacles_copy.json')
        with open(obs_fp) as f: 
            obs_json = json.load(f)
        
        for elem in obs_json['dynamic']: 
            for i in range(len(elem['vertices'])): 
                elem['vertices'][i] = [elem['vertices'][i][0]+elem['vel'][0]*1,elem['vertices'][i][1]+elem['vel'][1]*1] 

        with open(obs_fp, mode='w') as f: 
            json.dump(obs_json,f)

        self.count += 1

    def get_static_obstacles(self): 
        obs_fp = os.path.join(str(Path(__file__).parent.parent.parent), 'data', 'obstacles.json')
        with open(obs_fp) as f: 
            obs_json = json.load(f)
        
        static_obs = []
        for elem in obs_json['static']:
            static_obs.append(elem)
        static_original_obs = np.array(static_obs.copy(), dtype=object)
        static_padded_obs = self.pad_obstacles(static_original_obs)
        static_obstacles_shapely = self.obs_as_shapely_polygon(static_padded_obs)

        unexpected_obs = [] 
        if self.count > 30:    
            for elem in obs_json['unexpected']: 
                unexpected_obs.append(elem['vertices'])
    

        unexpected_original_obs = unexpected_obs.copy()
        unexpected_padded_obs = self.pad_obstacles(unexpected_original_obs)
        unexpected_obstacles_shapely = self.obs_as_shapely_polygon(unexpected_padded_obs)
        return static_original_obs, static_padded_obs, static_obstacles_shapely, unexpected_original_obs, unexpected_padded_obs, unexpected_obstacles_shapely


    def pad_obstacle(self, obstacle, inflation):
        if obstacle == []: return []

        self.inflator.Clear()
        self.inflator.AddPath(obstacle, pyclipper.JT_MITER, pyclipper.ET_CLOSEDPOLYGON)
        inflated_obstacle = pyclipper.scale_from_clipper(self.inflator.Execute(inflation))[0]
        return inflated_obstacle
    
    def pad_obstacles(self, obs, dynamic=False, decrease=False, scale_dist=None):
        """Pads the obstacles with self.config.vehicle_width or scale_dist 

        Args:
            obs (list of lists): list of lists of vertices describing the obstacles
            dynamic (bool, optional): If the obstacle is dynamic then obs is actually list of lists of vertices. Treat it accordingly. Defaults to False.
            decreate (bool, optional): If the padded obstacle should be smaller. For example boundary. Defaults to False.

        Returns:
            list: Either a list of list of vertices or list of vertices depending on dynamic
        """
        if scale_dist is None:
            scale_dist = self.config.vehicle_width


        if decrease:
            inflation = pyclipper.scale_to_clipper(-scale_dist)
        else:
            inflation = pyclipper.scale_to_clipper(scale_dist)
        
        if dynamic:
            inflated_obstacles = []
            for i, ob in enumerate(obs):
                tmp_list = []
                for j, ob_ in enumerate(ob):
                    obstacle = pyclipper.scale_to_clipper(ob_)
                    inflated_obstacle = self.pad_obstacle(obstacle, inflation)
                    inflated_obstacle.reverse() # obstacles are ordered clockwise
                    tmp_list.append(inflated_obstacle)

                inflated_obstacles.append(tmp_list)

        else:    
            inflated_obstacles = []        
            for ob in obs:
                obstacle = pyclipper.scale_to_clipper(ob)
                inflated_obstacle = self.pad_obstacle(obstacle, inflation)
                inflated_obstacle.reverse() # obstacles are ordered clockwise
                inflated_obstacles.append(inflated_obstacle)

        res = np.array(inflated_obstacles) if len(inflated_obstacles) == 1 else np.array(inflated_obstacles, dtype='object')
        return res
    
    def obs_as_shapely_polygon(self, obstacles):
        return [shapely.geometry.Polygon(ob) for ob in obstacles]

    def obstacles_as_triangles(self, obstacles):
        return [self.obstacle_as_triangle(ob) for ob in obstacles]

    def obstacle_as_triangle(self, obstacle):
        start_time = perf_counter_ns()

        def mark_domains(ct, start_face, index, edge_border, face_info):
            if face_info[start_face].nesting_level != -1:
                return
            queue = [start_face]
            while queue != []:
                fh = queue[0]     # queue.front
                queue = queue[1:]  # queue.pop_front
                if face_info[fh].nesting_level == -1:
                    face_info[fh].nesting_level = index
                    for i in range(3):
                        e = (fh, i)
                        n = fh.neighbor(i)
                        if face_info[n].nesting_level == -1:
                            if ct.is_constrained(e):
                                edge_border.append(e)
                            else:
                                queue.append(n)

        def mark_domain(cdt):
            """Find a mapping that can be tested to see if a face is in a domain
            Explore the set of facets connected with non constrained edges, 
            and attribute to each such set a nesting level.
            We start from the facets incident to the infinite vertex, with a
            nesting level of 0. Then we recursively consider the non-explored
            facets incident to constrained edges bounding the former set and
            increase the nesting level by 1.
            Facets in the domain are those with an odd nesting level.
            """
            face_info = {}
            for face in cdt.all_faces():
                face_info[face] = FaceInfo2()
            index = 0
            border = []
            mark_domains(cdt, cdt.infinite_face(), index+1, border, face_info)
            while border != []:
                e = border[0]       # border.front
                border = border[1:]  # border.pop_front
                n = e[0].neighbor(e[1])
                if face_info[n].nesting_level == -1:
                    lvl = face_info[e[0]].nesting_level+1
                    mark_domains(cdt, n, lvl, border, face_info)
            return face_info

        def insert_polygon(cdt, polygon):
            if not polygon:
                return

            handles = [cdt.insert(polypt) for polypt in polygon]
            for i in range(len(polygon)-1):
                cdt.insert_constraint(handles[i], handles[i+1])
            cdt.insert_constraint(handles[-1], handles[0])

        def get_coords(cdt, face_info):
            coords = []
            for face in cdt.finite_faces():
                if face_info[face].in_domain():
                    coords.append([(face.vertex(i).point().x(), face.vertex(i).point().y()) for i in range(3)])

            return coords



        # Make sure the last coordinate isn't the same as the first
        obstacle = obstacle if not np.all(obstacle[0] == obstacle[-1]) else obstacle[:-1]

        polygon = [Point_2(coord[0], coord[1]) for coord in obstacle]

        cdt = Constrained_Delaunay_triangulation_2()
        insert_polygon(cdt, polygon)

        # Mark facest that are inside the domain bounded by the polygon
        face_info = mark_domain(cdt)

        # Extract the triangles
        triangles = get_coords(cdt, face_info)

        self.times['obstacle_as_triangle'].append(perf_counter_ns() - start_time)
        return triangles
    

    def bounds_to_obstacles_and_convex_hull(self, boundary_original):
        """[summary]

        Args:
            boundary_original ([type]): [description]

        Returns:
            List of list: tuple coordinates of obstacles
            List: tuple coordinates of the convex boundary
        """
        org = shapely.geometry.Polygon(boundary_original)
        convex_part = org.convex_hull
        # Get the difference between original boundary and convex hull
        nonconvex_part = convex_part.symmetric_difference(org)

        extra_obs_list_coords = []
        if nonconvex_part.geom_type == 'MultiPolygon': # extract polygons out of multipolygon
            for polygon in nonconvex_part:
                x,y = polygon.exterior.xy
                coords = []
                #Probably exist an easier way of putting coordinates into lists
                for i in range(len(x)):
                    coords.append((x[i],y[i]))
                # Make sure the last coordinate isn't the same as the first
                coords = coords if not np.all(coords[0] == coords[-1]) else coords[:-1]
                extra_obs_list_coords.append(coords)
        elif nonconvex_part.geom_type =='Polygon' and nonconvex_part.area > 0 :
            x,y = nonconvex_part.exterior.xy
            coords = []
            for i in range(len(x)):
                coords.append((x[i],y[i]))
            # Make sure the last coordinate isn't the same as the first
            coords = coords if not np.all(coords[0] == coords[-1]) else coords[:-1]
            extra_obs_list_coords.append(coords)
        else:
            extra_obs_list_coords = None

        # Make boundary into tuple list 
        x_boundary, y_boundary = convex_part.exterior.xy
        convex_boundary = []
        for i in range(len(x_boundary)):
            convex_boundary.append((x_boundary[i],y_boundary[i]))
        
        # Make sure the last coordinate isn't the same as the first
        convex_boundary = convex_boundary if not np.all(convex_boundary[0] == convex_boundary[-1]) else convex_boundary[:-1]
        return extra_obs_list_coords, convex_boundary

    def dist2closest_dyn_obs(self, point, poly_list):
        """[summary]

        Args:
            point ([type]):  ( tuple )
            poly_list ([type]): poly_list: [ob1 :[ob1_t0, ob1_t1 ,...], ob2 : [ob2_t0,]]

        Returns:
            [type]: minimum dist from point to obs in list 
        """
        current_pos = shapely.geometry.Point(point)
        min_dist = None
        for obstacle in poly_list: 
            # only consider objects at time t = 0 
            dist = current_pos.distance(obstacle[0]) 
            
            if min_dist is None:
                min_dist = dist
            elif dist < min_dist:
                min_dist = dist

        return min_dist


    @staticmethod
    def plot_obs(c, obs, ax, where_to_store_paintings):
        for ob in obs:
            x = [x for x, y in ob]
            y = [y for x, y in ob]
            x.append(x[0])
            y.append(y[0])

            tmp, = ax.plot(x, y, c=c)
            where_to_store_paintings.append(tmp)

    def solve_mvee(self):
        x = symbols('x')
        y = symbols('y')

        x0 = symbols('x0')
        y0 = symbols('y0')
        a = symbols('a')
        b = symbols('b')
        d = symbols('c')
        e = symbols('d')

        x_bar = x-x0
        y_bar = y-y0
        eq = 1 - (x_bar*(a*x_bar+b*y_bar) + y_bar*(d*x_bar+e*y_bar))
        sol = solve(eq,x)

        syms = [x0, y0, a, b, d, e, y]
        sol_v1 = sol[0]
        sol_v2 = sol[1]

        # Create functions to quickly evaluate the equations
        sol_v1 = lambdify(syms, sol_v1, "numpy")
        sol_v2 = lambdify(syms, sol_v2, "numpy")
        self.mvee_solutions = [sol_v1, sol_v2]

    def calc_point_on_mvee(self, c, A):

        # Evaluate to see if the coordiante is on the ellipse
        vals = np.hstack((c,A.reshape(-1)))
        vals = np.repeat(vals.reshape(1,-1), self.plot_config['obs_dyn_n_points'], axis=0)
        ys = np.linspace(c[1]-self.plot_config['obs_dyn_max_size']/2, c[1]+self.plot_config['obs_dyn_max_size']/2, num=self.plot_config['obs_dyn_n_points'])
        ys_flipped = np.flip(ys)
        vals1 = np.hstack((vals, ys.reshape(-1,1)))
        vals2 = np.hstack((vals, ys_flipped.reshape(-1,1)))
        solution_1 = np.array([])
        solution_2 = np.array([])
        with ignore_warnings(RuntimeWarning):
            for i in range(vals1.shape[0]):
                s1 = self.mvee_solutions[0](*vals1[i])
                solution_1 = np.hstack((solution_1, s1))
                s2 = self.mvee_solutions[1](*vals2[i])
                solution_2 = np.hstack((solution_2, s2))

        # Combine the results
        xs = np.hstack((solution_1, solution_2))
        ys = np.hstack((ys, ys_flipped))

        # Prune the nans from ys and xs
        extract_rule = np.logical_not(np.isnan(xs))
        ys = ys[extract_rule]
        xs = xs[extract_rule]

        return xs, ys

    def polygon_to_coord_plotable(self, data, mvee=True, polygon=False, panoc=None):
        if mvee and not polygon:
            Ac = data
            assert Ac.shape == (6,), f"The shape of the data equations must be (6,) when mvee=True, it was: {Ac.shape}. It must be so that Ac = [A[0,0] A[0,1] A[1,0] A[1,1] c[0], c[1]]."
        elif polygon and not mvee:
            A, c  = panoc.obstacle_as_ellipse(data)
            Ac = np.hstack((A.reshape(-1), c))
        else:
            raise ValueError("Either mvee or polygon must be true. Not both, not none.")
            

        xs, ys = self.calc_point_on_mvee(Ac[4:], Ac[:4])
        data  = np.vstack((xs,ys)).T
        return data

    def plot(self, boundary_padded,static_original_obs, static_padded_obs, unexpected_original_obs, unexpected_padded_obs, dynamic_original_obs,dynamic_padded_obs, panoc):
        start_time = perf_counter_ns()
        
        # Plot all obstacles unpadded
        if self.plot_config['enable_original_obs']:
            obs = []
            if np.all(static_original_obs.shape):
                obs += [ob for ob in static_original_obs]
            if np.all(len(unexpected_original_obs)):
                obs += [ob for ob in unexpected_original_obs]
            if np.all(dynamic_original_obs.shape):
                obs += [ob for ob in dynamic_original_obs[:, 0, :, :]]
                # Plot the future positions of the dynamic obstacles
                if self.plot_config['plot_future_dyn_obs']:
                    obs += [ob for ob in dynamic_original_obs[:, self.config.n_hor -1, :, :]]
                
            try: self.plot_queues['obstacles_original'].put_nowait(obs)
            except Full: pass


        # Plot all obstacles padded
        obs = []
        if np.all(static_padded_obs.shape):
            obs += [ob for ob in  static_padded_obs]
        if np.all(unexpected_padded_obs.shape):
            obs += [ob for ob in  unexpected_padded_obs]
        if np.all(dynamic_padded_obs.shape):
            if not self.plot_config['enable_ellipses']:
                # Plot the polygon representation of the dynamic obstacles
                obs += [ob for ob in dynamic_padded_obs[:, 0, :, :]]
                # Plot the future positions of the dynamic obstacles
                if self.plot_config['plot_future_dyn_obs']:
                    obs += [ob for ob in dynamic_padded_obs[:, self.config.n_hor -1, :, :]]
            

            if self.plot_config['enable_ellipses']:
                # Plot the ellipse representation of the dynamic obstacles
                obs += [self.polygon_to_coord_plotable(ob, mvee=False, polygon=True, panoc=panoc) for ob in dynamic_padded_obs[:, 0, :, :]]
                # Plot the future positions of the dynamic obstacles
                if self.plot_config['plot_future_dyn_obs']:
                    obs += [self.polygon_to_coord_plotable(ob, mvee=False, polygon=True, panoc=panoc) for ob in dynamic_padded_obs[:, self.config.n_hor -1, :, :]]

        try: self.plot_queues['obstacles_padded'].put_nowait(obs)
        except Full: pass

        # Plot boundary
        data = np.vstack((boundary_padded, boundary_padded[0])).T
        try: self.plot_queues['boundary'].put_nowait(data)
        except Full: pass


        self.times['plot'].append(perf_counter_ns() - start_time)

    def get_closest_static_obstacles(self, cur_pos):
        """Returns the shapely representation of the closest static and unexpected obstacles to cur_pos. How many is determined by self.config.n_obs_of_each_vertices.

        Args:
            cur_pos ([type]): [description]

        Returns:
            dictionary: Dictionary with keys: [self.config.min_vertices:self.config.max_vertices] which contain the closest obstacles
        """
        start_time = perf_counter_ns()
        _,_, static_obstacles_shapely, _,_, unexpected_obstacles_shapely = self.get_static_obstacles()
        # Convert all concave objects into convex ones
        n_hornings = {n:[] for n in range(self.config.min_vertices, self.config.max_vertices+1)}
        
        # Function for if any of the polygons are too high dimensional, reduce them by turning them into triangles
        def reduce_dim(ob):
            return concave_to_convex(ob)
        
        def concave_to_convex(ob):
            ob = self.obstacle_as_triangle(np.array(ob.exterior.xy).T)
            ob = self.obs_as_shapely_polygon(ob)
            # pad the obstacles
            ob_pad = pad_triangles(ob)
            return ob_pad

        # Function for padding the triangles a bit
        def pad_triangles(ob):
            ob_pad = []
            for ob_ in ob:
                # Pad them so that the smallest increase is 5 mm
                x,y = ob_.exterior.xy
                x = np.array(x)
                y = np.array(y)
                xdist = np.abs(x.max()-x.min())
                xfact = (xdist + 0.05)/xdist
                ydist = np.abs(y.max()-y.min())
                yfact = (ydist + 0.05)/ydist
                ob_pad.append(shapely.affinity.scale(ob_, xfact=xfact, yfact=yfact, origin='center'))
            return ob_pad

        # Sort the n_nornings
        def sort_hornings(obs):
            for ob in obs:
                # Check if the polygon is concave and if so convert it into triangles and pad the triangles a bit
                if ob.area < ob.convex_hull.area:
                    ob_convex = concave_to_convex(ob)
                    n_hornings[3].extend(ob_convex) 
                # If it's too high dimensional, reduce it. They will be reduced to triangles
                elif len(ob.exterior.xy[0]) > self.config.max_vertices:
                    ob_reduced = reduce_dim(ob)
                    n_hornings[3].extend(ob_reduced) 
                else: 
                    n = len(ob.exterior.xy[0]) - 1
                    n_hornings[n].append(ob)
        
        sort_hornings(static_obstacles_shapely)
        sort_hornings(unexpected_obstacles_shapely)
        
        # Find the closest obstacles of each horning type
        n_closest = {n:[] for n in range(self.config.min_vertices, self.config.max_vertices+1)}
        cur_pos = shapely.geometry.Point(cur_pos)
        for n in n_closest:
            obs = n_hornings[n]
            if len(obs) == 0: continue
            
            # Find the obs with the smallest distance
            dist = {}
            for ob in obs:
                d = ob.exterior.distance(cur_pos)
                # See if there is another shape with the exact same distance, if so add to it
                try:
                    dist[d].append(ob)
                except KeyError:
                    dist[d] = [ob]

            max_allowed_obstacles = self.config.n_obs_of_each_vertices #TODO: Add special case for triangles
            if len(dist) <= max_allowed_obstacles: 
                smallest_dist = dist.keys()
            else:
                smallest_dist = np.partition(list(dist.keys()), max_allowed_obstacles)[:max_allowed_obstacles]
                smallest_dist = np.sort(list(smallest_dist))

            # If there are more than one polygon with the same distance it's possible that 
            # len(n_closest[n]) is bigger than the allowed number of obstacles. So we must remove if there are too many
            closest = []
            for d in smallest_dist: closest.extend(dist[d])
            n_closest[n] = closest[:max_allowed_obstacles] 


        self.times['get_closest_obs'].append(perf_counter_ns() - start_time)
        return n_closest

    def get_closest_dynamic_obstacles(self, cur_pos):
        """Returns the equation representation of the closest dynamic obstacles to cur_pos for each timestep in self.config.n_hor.
        Only the obstacles which are closest in the first time step are returned. So if one obstacle comes closer in the future it 
        still isn't considered.
        How many is determined by self.config.n_obs_of_each_vertices.

        Args:
            cur_pos ([type]): [description]

        Returns:
            List: List with the variables A and c in (x-c).T * A * (x-c) = 1, flattened, as [[A0 c0], [A1 c1]...]
        """
        #TODO: Examine if there is a bug when two obstacles are on the exact same distance from eachother
        start_time = perf_counter_ns()
        _, dynamic_padded_obs, dynamic_obstacles_shapely = self.get_dynamic_obstacles()

        # Find the closest obstacles
        obs = dynamic_padded_obs
        if not np.all(obs.shape): return []
        
        shapely_obs = np.array(dynamic_obstacles_shapely)[:, 0]

        cur_pos = shapely.geometry.Point(cur_pos)
        dist = {ob.exterior.distance(cur_pos):i for i, ob in enumerate(shapely_obs)}
        if len(shapely_obs) <= self.config.n_dyn_obs:
            smallest_dist = range(0, len(shapely_obs))
        else:
            smallest_dist = np.argpartition(list(dist.keys()), self.config.n_dyn_obs)[:self.config.n_dyn_obs]

        self.times['get_closest_obs'].append(perf_counter_ns() - start_time)
        return [dynamic_obstacles_shapely[i] for i in smallest_dist] # It should also return it's padded polygon counterpart
        

        