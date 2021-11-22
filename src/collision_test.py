# THIS IS JUST A FILE FOR TESTING CODE BEFORE IMPLEMENTING IT IN THE MAIN SCRIPT

# import json
# import os
# from pathlib import Path

# from path_planner.obstacle_handler import ObstacleHandler

# file_path = Path(__file__)

# obs_original = os.path.join(str(file_path.parent.parent), 'data', 'obstacles.json')
# with open(obs_original) as f:
#     distros_dict = json.load(f)

# for distro in distros_dict:
#     print(distro())


# # import Point, Polygon
# from sympy import Point, Polygon
  
# # creating points using Point()
# p1, p2, p3, p4 = map(Point, [(0, 0), (1, 0), (5, 1), (0, 1)])
# p5, p6, p7 = map(Point, [(3, 2), (1, -1), (0, 2)])

# poly_test = Polygon((0,0),(1,2),(3,4))
# print(poly_test)

  
# # creating polygons using Polygon()
# poly1 = Polygon(p1, p2, p3, p4)
# poly2 = Polygon(p5, p6, p7)


# isIntersection1 = poly_test.intersection(poly1)


# # using intersection()
# isIntersection = poly1.intersection(poly2)
  
# print(isIntersection)


from shapely.geometry import LineString
from shapely.geometry import Point

p = Point(5,5)
c = p.buffer(3).boundary #A circle with origin in 5,5 and radius 3.
l = LineString([(0,0), (10, 10)]) #A line through 0,0 and 10,10
i = c.intersection(l)
print(i)
print(c.intersects(l))  #Returns true if the two geometries intersect

print (i.geoms[0].coords[0])
# (2.8786796564403576, 2.8786796564403576)

print (i.geoms[1].coords[0])
# (7.121320343559642, 7.121320343559642)

from shapely.geometry import Polygon
polygon = Polygon([(1, 3), (4, 4), (3, 1)])
polygon2 = Polygon([(6,2),(3,2),(3,3),(6,3)])
print(polygon)
print(polygon2.area)
# 0.5
polygon.length
# 3.4142135623730949
print(polygon.intersection(polygon2))
print(polygon.intersects(polygon2))



import matplotlib.pyplot as plt

def plot_polygons(polygons):
    # polygons is a list of Polygon objects
    x = []
    y = []
    for p in polygons:
        x_temp,y_temp = p.exterior.xy
        # x.append(x_temp)
        # y.append(y_temp)
    
        plt.plot(x_temp,y_temp)
    plt.show()

# plt.figure()
# x,y = polygon.exterior.xy
# a,b = polygon2.exterior.xy
# plt.plot(x,y)
# plt.plot(a,b)
# # plt.plot(polygon.exterior.xy)
# plt.show()

plot_polygons([polygon2,polygon])

# x=[[1,2,3,4],[1,2,3,4],[1,2,3,4],[1,2,3,4]]
# y=[[1,2,3,4],[1,2,3,4],[1,2,3,4],[1,2,3,4]]
# for i in range(len(x)):
#     plt.figure()
#     plt.plot(x[i],y[i])
#     # Show/save figure as desired.
#     plt.show()
# # Can show all four figures at once by calling plt.show() here, outside the loop.
# #plt.show()



# # x axis values
# x = [1,2,3]
# # corresponding y axis values
# y = [2,4,1]
 
# # plotting the points
# plt.plot(x, y)
 
# # naming the x axis
# plt.xlabel('x - axis')
# # naming the y axis
# plt.ylabel('y - axis')
 
# # giving a title to my graph
# plt.title('My first graph!')
 
# # function to show the plot
# plt.show()