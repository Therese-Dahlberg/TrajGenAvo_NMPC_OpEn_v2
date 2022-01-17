# THIS IS JUST A FILE FOR TESTING CODE BEFORE IMPLEMENTING IT IN THE MAIN SCRIPT.

import json
import os
from pathlib import Path
import smallest_enclosing_circle as sec
import minimal_enclosing_circle as mec

import numpy as np
import random

from path_planner.obstacle_handler import ObstacleHandler

file_path = Path(__file__)

#Get the path to the json file
obs_original = os.path.join(str(file_path.parent.parent), 'data', 'obstacles.json')
with open(obs_original) as f:
    distros_dict = json.load(f)

#Prints the "keys" in the dictionaries in the json file. In this case "static", "unexpected" and "dynamic".
for distro in distros_dict:
    print(distro)


static_obs = []
unexpected_obs = []
for elem in distros_dict['static']:
    static_obs.append(elem)
    print("static ",static_obs)

for elem in distros_dict['unexpected']:
    unexpected_obs.append(elem['vertices'])
    print("unexpected ",unexpected_obs)


# obs_original = os.path.join(str(file_path.parent.parent), 'data', 'obstacles.json')
# obs_copy     = os.path.join(str(file_path.parent.parent), 'data', 'obstacles_copy.json')
# with open(obs_original) as f: 
#     obs_json = json.load(f)
# with open(obs_copy, mode='w') as f: 
#     json.dump(obs_json,f)

# for distro in obs_json:
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
    plt.axis('equal')
    plt.show()

# plt.figure()
# x,y = polygon.exterior.xy
# a,b = polygon2.exterior.xy
# plt.plot(x,y)
# plt.plot(a,b)
# # plt.plot(polygon.exterior.xy)
# plt.show()

# plot_polygons([polygon2,polygon])

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


### TESTING MINIMUM BOUNDING CIRCLE ###
import matplotlib.pyplot as plt
import matplotlib.collections as mplc
import libpysal as ps
from shapely import geometry as sgeom
import descartes as des
import pointpats

data = ps.io.open(ps.examples.get_path('columbus.shp')).read()
chains = [chain.parts[0] for chain in data]

points = chains[0]
# print(points)

poly = sgeom.Polygon(points)
center = poly.centroid
circle = Point(center).buffer(1)
print(poly.boundary)
print("center", poly.distance(center))
# print(circle.exterior.xy)
# print("circle", circle)
# plot_polygons([circle,poly])
# print(type(static_obs))
# print(type(points))
obs = sgeom.Polygon(unexpected_obs[0])
origin_obs = obs.centroid
# origin_obs.distance(static_obs[0][0])

def minimum_bounding_circle(polygon_vertices,origin):

    print("polygon vertices: ", polygon_vertices)

    r = 0
    for p in polygon_vertices:
        print("p: ", p)
        point = sgeom.Point(p)
        print("point converted: ",point)
        dist = origin.distance(point)
        if dist > r:
            r = dist
            print("radius: ",r)
    mbc = sgeom.Point(origin).buffer(r)
    return mbc

mbc = minimum_bounding_circle(unexpected_obs[0],origin_obs)
# plot_polygons([obs,mbc])


mbc2 = minimum_bounding_circle(points,center)
# plot_polygons([poly,mbc2])

poly_static = sgeom.Polygon(static_obs[0])
origin_static = poly_static.centroid
mbc3 = minimum_bounding_circle(static_obs[0],origin_static)
# plot_polygons([poly_static,mbc3])


#radius is then from the center to the farthest away vertex
# print(poly)

# (radius,center), _, _, _ = pointpats.skyum(points)
# mbc_poly = sgeom.Point(*center).buffer(radius)
# sgeom.Point().buffer(radius)

# fig = plt.figure(figsize=(10,10))
# ax = fig.add_subplot(111)
# ax.set_xlim(8, 10)
# ax.set_ylim(13,16)
# ax.plot([p[0] for p in points], [p[-1] for p in points], 'r')
# ax.add_patch(des.PolygonPatch(mbc_poly, fc='white', ec='black'))
# chull = pointpats.hull(points)
# ax.plot([p[0] for p in chull], [p[-1] for p in chull], 'm')
# ax.plot([p[0] for p in constraints], [p[-1] for p in constraints], '^b')
# ax.plot([p[0] for p in inset], [p[-1] for p in inset], 'ob')
# ax.plot([p[0] for p in removed], [p[-1] for p in removed], 'xb')
# plt.show()

def plot_points_and_circle(points,circle):
    # polygons is a list of Polygon objects

    x_temp,y_temp = circle.exterior.xy
    # print("test here")
    # print(x_temp,y_temp)
        # x.append(x_temp)
        # y.append(y_temp)
    x = []
    y = []
    for p in points:
        x.append(p[0])
        y.append(p[1])
    plt.plot(x_temp,y_temp)
    plt.scatter(x,y)
    plt.axis('equal')
    plt.show()





circle_1 = sec.make_circle(static_obs[0])
print(circle_1)

print("trhgetg", unexpected_obs[0])
circle_2 = sec.make_circle(unexpected_obs[0])
print(circle_2)
circle_2 = sgeom.Point([circle_2[0],circle_2[1]]).buffer(circle_2[2])

# plot_polygons([circle_2,obs])

print("un obs: ", unexpected_obs[0])
circle_3 = mec.minimum_enclosing_circle(unexpected_obs[0])
circle_3a = sgeom.Point(circle_3[0]).buffer(circle_3[1])
# plot_polygons([circle_3a,obs])

test_points = np.random.uniform(0, 4, size=(50,2))
list_points = test_points.tolist()
# print("test points", test_points, list_points)





points = [[1.0,4.0], [2.0,5.0], [4.0,5.0], [5.0, 4.0], [5.0, 2.0], [4.0, 1.0], [2,1], [1,2]]
# points = [[0.0,0.0],[0.0,1.0], [4.0,1.0],[4.0,0.0]]
polygon4 = sgeom.Polygon(points)
circle_4 = sec.make_circle(points)
print("c4", circle_4)
circle_4a = sgeom.Point(circle_4[0],circle_4[1]).buffer(circle_4[2])


points2 = [[5.1, 4.0], [5.6, 4.5], [6.6, 3.5], [6.1, 3.0]] #No collision
points2 = [[5.9, 2.0], [5.9, 4.0], [7.0, 4.0],[7.0, 2.0]] 
polygon5 = sgeom.Polygon(points2)
circle_5 = sec.make_circle(points2)
print("c5", circle_5)
circle_5a = sgeom.Point(circle_5[0],circle_5[1]).buffer(circle_5[2])



# plot_points_and_circle(points,circle_4a)
# plot_polygons([polygon4,circle_4a,polygon5,circle_5a])


import numpy
import miniball #Use for Welzl's Algorithm!
import math

pi = math.pi

def PointsInCircum(o_x,o_y,r,n=100):
    return [(math.cos(2*pi/n*x)*r + o_x,math.sin(2*pi/n*x)*r+o_y) for x in range(0,n+1)]

S = numpy.random.randn(100,2)
print("type of S ", type(S))
C,r2 = miniball.get_bounding_ball(S)

S_list = S.tolist()

print(C,r2)
r = math.sqrt(r2)

circle_6a = sgeom.Point(C[0],C[1]).buffer(r)

# plot_points_and_circle(S_list,circle_6a)


points = [[1.0,4.0], [2.0,5.0], [4.0,5.0], [5.0, 4.0], [5.0, 2.0], [4.0, 1.0], [2,1], [1,2]]
# points = [[0.0,0.0],[0.0,1.0], [4.0,1.0],[4.0,0.0]]
polygon4 = sgeom.Polygon(points)
points_numpy = numpy.array(points)
circle_4,r2_2 = miniball.get_bounding_ball(points_numpy)
r_2 = math.sqrt(r2_2)
print("c4", circle_4)
circle_4a = sgeom.Point(circle_4[0],circle_4[1]).buffer(r_2)

# plot_points_and_circle(points,circle_4a)
# plot_polygons([circle_4a,polygon4])




test,radius_2 = miniball.get_circumsphere(points_numpy)
# print("ffjjopjiji")
# print(test)

radius = math.sqrt(radius_2)

plot_points = PointsInCircum(test[0],test[1],radius,10)
print("gen points", plot_points)

plot_points_and_circle(plot_points,circle_4a)


# import pyqtgraph as pg
# import numpy as np
# import sys
# from PyQt4 import QtGui

# app = QtGui.QApplication(sys.argv)  # Create QApplication ***

# x = np.arange(1000)
# y = np.random.normal(size=(3, 1000))
# plotWidget = pg.plot(title="Three plot curves")
# for i in range(3):
#     plotWidget.plot(x, y[i], pen=(i, 3))

# # Start Qt event loop unless running in interactive mode or using pyside.
# if __name__ == '__main__':
#     if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
#         app.exec_()  # Start QApplication event loop ***





# import numpy as np
# import pyqtgraph as pg
# image = np.random.normal(size=(500, 400))
# plt1 = pg.PlotWidget()
# plt1_imageitem = pg.ImageItem(image)
# plt1.addItem(plt1_imageitem)
# roi_circle = pg.CircleROI([250, 250], [120, 120], pen=pg.mkPen('r',width=2))
# # roi_circle.sigRegionChanged.connect(circle_update)
# plt1.addItem(roi_circle)
# plt1.show()

# import pyqtgraph as pg
# from PyQt5 import QtCore, QtGui, QtWidgets

# win = pg.GraphicsLayoutWidget(show=True, title="Plotting")
# p = win.addPlot(title='test')

# p_ellipse = pg.QtGui.QGraphicsEllipseItem(0, 0, 10, 10)  # x, y, width, height
# p_ellipse2 = pg.ROI([1, 1], [27, 28], pen='y')
# p_ellipse3 = pg.EllipseROI([0, 0], [10, 10], pen='y')
# # p_ellipse.setPen(pg.mkPen((0, 0, 0, 100)))
# p_ellipse.setPen(pg.mkPen('g', width=0))
# p_ellipse.setBrush(pg.mkBrush((0, 0, 0)))

# p.addItem(p_ellipse)
# p.addItem(p_ellipse2)
# p.addItem(p_ellipse3)
# win.show()


# import pyqtgraph as pg
# plot = pg.plot()

# e1 = pg.QtGui.QGraphicsEllipseItem(0, 0, 4, 4)
# # MUST have width=0 here, or use a non-cosmetic pen:
# e1.setPen(pg.mkPen('r', width=0))
# e1.setFlag(e1.ItemClipsChildrenToShape)
# plot.addItem(e1)

# e2 = pg.QtGui.QGraphicsEllipseItem(2, 2, 4, 4)
# e2.setPen(pg.mkPen('g'))
# e2.setParentItem(e1)




# w = pg.GraphicsLayoutWidget(QtWidgets.QGraphicsView(QtWidgets.QWidget(QtWidgets.QMainWindow())))

# x = [1, 2, 3, 4]
# y = [1, 2, 3, 4]

# plot_item = w.addPlot()
# scatter_item = pg.ScatterPlotItem()
# scatter_item.setData(x, y)
# plot_item.addItem(scatter_item)

