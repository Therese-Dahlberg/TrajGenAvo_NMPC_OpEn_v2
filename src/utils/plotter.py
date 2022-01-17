from copyreg import remove_extension
import numpy as np
import multiprocessing
from queue import Empty
import math

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
from shapely.geometry import box as Box
from shapely.geometry import Polygon
import matplotlib.pyplot as plt
# from src.main import collisionDetection
# import src.main

def start_plotter(config, plot_config, aut_test_config=None, width=800, height=600):
    plot_queues_map  = {'master_path':multiprocessing.Queue(maxsize=1),

                'master_start':multiprocessing.Queue(maxsize=1), 
                'master_end':multiprocessing.Queue(maxsize=1), 
                'slave_path':multiprocessing.Queue(maxsize=1), 
                'slave_start':multiprocessing.Queue(maxsize=1), 
                'slave_end':multiprocessing.Queue(maxsize=1), 
                'obstacles_original':multiprocessing.Queue(maxsize=1), 
                'obstacle_bc':multiprocessing.Queue(maxsize=1),  #bounding circele around obstacle
                # 'obstacles_padded':multiprocessing.Queue(maxsize=1), 
                'closest_obstacles':multiprocessing.Queue(maxsize=1), 
                'traversed_path':multiprocessing.Queue(maxsize=1), 
                'planned_path':multiprocessing.Queue(maxsize=1), 
                'planned_trajectory_master':multiprocessing.Queue(maxsize=1), 
                'planned_trajectory_slave':multiprocessing.Queue(maxsize=1), 
                'nodes':multiprocessing.Queue(maxsize=1), 
                'boundry':multiprocessing.Queue(maxsize=1), 
                # 'look_ahead':multiprocessing.Queue(maxsize=1), 
                'trajectory_master':multiprocessing.Queue(maxsize=1), 
                'trajectory_slave':multiprocessing.Queue(maxsize=1), 
                'line_vertices_master':multiprocessing.Queue(maxsize=1), 
                'line_vertices_slave':multiprocessing.Queue(maxsize=1), 
                'ref_point_master':multiprocessing.Queue(maxsize=1), 
                'ref_point_slave':multiprocessing.Queue(maxsize=1), 
                'search_sector':multiprocessing.Queue(maxsize=1), 
                'object_collision': multiprocessing.Queue(maxsize=1),
                'object_safe': multiprocessing.Queue(maxsize=1), # the carried object which needs to be plotted later on
                'cargo_bc': multiprocessing.Queue(maxsize=1),
                'object_collision_1': multiprocessing.Queue(maxsize=1), # the carried object which needs to be plotted later on
                'object_collision_2': multiprocessing.Queue(maxsize=1),
                'object_collision_3': multiprocessing.Queue(maxsize=1),
                'object_collision_4': multiprocessing.Queue(maxsize=1),
                'object_collision_5': multiprocessing.Queue(maxsize=1),
                'object_safe_1': multiprocessing.Queue(maxsize=1), # the carried object which needs to be plotted later on
                'object_safe_2': multiprocessing.Queue(maxsize=1),
                'object_safe_3': multiprocessing.Queue(maxsize=1),
                'object_safe_4': multiprocessing.Queue(maxsize=1),
                'object_safe_5': multiprocessing.Queue(maxsize=1),
            } 
    
    plot_queues_data  = {'master_lin_vel':multiprocessing.Queue(maxsize=1), 
               'master_lin_acc':multiprocessing.Queue(maxsize=1), 
               'master_lin_jerk':multiprocessing.Queue(maxsize=1), 
               'master_ang_acc':multiprocessing.Queue(maxsize=1), 
               'master_ang_vel':multiprocessing.Queue(maxsize=1), 
               'master_ang_jerk':multiprocessing.Queue(maxsize=1), 

               'slave_lin_vel':multiprocessing.Queue(maxsize=1), 
               'slave_lin_acc':multiprocessing.Queue(maxsize=1), 
               'slave_lin_jerk':multiprocessing.Queue(maxsize=1), 
               'slave_ang_acc':multiprocessing.Queue(maxsize=1), 
               'slave_ang_vel':multiprocessing.Queue(maxsize=1), 
               'slave_ang_jerk':multiprocessing.Queue(maxsize=1), 

               'constraint':multiprocessing.Queue(maxsize=1),
            }

    plot_queues_cost  = {'cost_master_lin_vel':multiprocessing.Queue(maxsize=1), 
               'cost_master_lin_acc':multiprocessing.Queue(maxsize=1),
               'cost_master_lin_jerk':multiprocessing.Queue(maxsize=1),
               'cost_master_ang_acc':multiprocessing.Queue(maxsize=1),
               'cost_master_ang_vel':multiprocessing.Queue(maxsize=1),
               'cost_master_ang_jerk':multiprocessing.Queue(maxsize=1),
               'cost_master_line_deviation':multiprocessing.Queue(maxsize=1),
               'cost_master_static_obs':multiprocessing.Queue(maxsize=1),
               'cost_master_dynamic_obs':multiprocessing.Queue(maxsize=1),
               'cost_master_ref_points':multiprocessing.Queue(maxsize=1),
               'cost_master_vel_ref':multiprocessing.Queue(maxsize=1),
               
               'cost_slave_lin_vel':multiprocessing.Queue(maxsize=1), 
               'cost_slave_lin_acc':multiprocessing.Queue(maxsize=1),
               'cost_slave_lin_jerk':multiprocessing.Queue(maxsize=1),
               'cost_slave_ang_acc':multiprocessing.Queue(maxsize=1),
               'cost_slave_ang_vel':multiprocessing.Queue(maxsize=1),
               'cost_slave_ang_jerk':multiprocessing.Queue(maxsize=1),
               'cost_slave_static_obs':multiprocessing.Queue(maxsize=1),
               'cost_slave_dynamic_obs':multiprocessing.Queue(maxsize=1),
               'cost_slave_ref_points':multiprocessing.Queue(maxsize=1),
               'cost_slave_vel_ref':multiprocessing.Queue(maxsize=1),

               'cost_constraint':multiprocessing.Queue(maxsize=1),

               'cost_circle':multiprocessing.Queue(maxsize=1),
               'cost_future_circles':multiprocessing.Queue(maxsize=1),
               
               'cost_future_master_lin_vel':multiprocessing.Queue(maxsize=1), 
               'cost_future_master_lin_acc':multiprocessing.Queue(maxsize=1),
               'cost_future_master_lin_jerk':multiprocessing.Queue(maxsize=1),
               'cost_future_master_ang_acc':multiprocessing.Queue(maxsize=1),
               'cost_future_master_ang_vel':multiprocessing.Queue(maxsize=1),
               'cost_future_master_ang_jerk':multiprocessing.Queue(maxsize=1),
               'cost_future_master_line_deviation':multiprocessing.Queue(maxsize=1),
               'cost_future_master_static_obs':multiprocessing.Queue(maxsize=1),
               'cost_future_master_dynamic_obs':multiprocessing.Queue(maxsize=1),
               'cost_future_master_ref_points':multiprocessing.Queue(maxsize=1),
               'cost_future_master_vel_ref':multiprocessing.Queue(maxsize=1),
               
               'cost_future_slave_lin_vel':multiprocessing.Queue(maxsize=1), 
               'cost_future_slave_lin_acc':multiprocessing.Queue(maxsize=1),
               'cost_future_slave_lin_jerk':multiprocessing.Queue(maxsize=1),
               'cost_future_slave_ang_acc':multiprocessing.Queue(maxsize=1),
               'cost_future_slave_ang_vel':multiprocessing.Queue(maxsize=1),
               'cost_future_slave_ang_jerk':multiprocessing.Queue(maxsize=1),
               'cost_future_slave_static_obs':multiprocessing.Queue(maxsize=1),
               'cost_future_slave_dynamic_obs':multiprocessing.Queue(maxsize=1),
               'cost_future_slave_ref_points':multiprocessing.Queue(maxsize=1),
               'cost_future_slave_vel_ref':multiprocessing.Queue(maxsize=1),

               'cost_future_constraint':multiprocessing.Queue(maxsize=1),

            }

    p1 = multiprocessing.Process(target=show_plot_map, args=(plot_queues_map, plot_queues_data, plot_queues_cost, width, height, config, plot_config, aut_test_config)) 
    p1.start() 
    return {**plot_queues_map, **plot_queues_data, **plot_queues_cost}, p1
    

def show_plot_map(plot_queues_map, plot_queues_data, plot_queues_cost, width, height, config, plot_config, aut_test_config):
    print("Show plot function")
    app = QtWidgets.QApplication([])
    pg.setConfigOptions(antialias=False) # True seems to work as well
    pg.setConfigOption('background', 'w')
    plottermap = PlotterMap(plot_queues_map, plot_queues_data, plot_queues_cost, width, height, config, plot_config, aut_test_config)
    plottermap.show()
    app.exec_()



class PlotterMap():
    def __init__(self, plot_queues_map, plot_queues_data, plot_queues_cost, width, height, config, plot_config, aut_test_config, parent=None):
        self.plot_queues_map = plot_queues_map
        self.plot_queues_data = plot_queues_data
        self.plot_queues_cost = plot_queues_cost
        self.width = width
        self.height = height
        self.config = config
        self.plot_config = plot_config
        self.aut_test_config = aut_test_config

        self.print_name = "[PLOTTER]"
        
        self.win_map = pg.GraphicsWindow(parent=parent)
        self.win_data = pg.GraphicsWindow(parent=parent)
        self.win_cost = pg.GraphicsWindow(parent=parent)

        # self.win_map = pg.GraphicsLayoutWidget(parent=parent)

        self.win_map.setLayout(QtWidgets.QVBoxLayout())
        self.win_map.addLabel()
        self.win_map.setWindowTitle('Map')
        self.data_layout = QtWidgets.QVBoxLayout()
        self.win_data.setLayout(self.data_layout)
        self.win_data.setWindowTitle('Data')
        self.win_data.addLabel()
        self.cost_layout = QtWidgets.QVBoxLayout()
        self.win_cost.setLayout(self.data_layout)
        self.win_cost.setWindowTitle('Costs')
        self.win_cost.addLabel()
        


        self.timer = QtCore.QTimer(self.win_map)
        self.timer.setInterval(100) # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.check_plot_queues)

        # Set up map plots
        self.map_plot = self.win_map.addPlot()
        self.map_plot.setAspectLocked()
        self.map_plot.setLabel('left', 'y', units='m')
        self.map_plot.setLabel('bottom', 'x', units='m')
        self.legend_map = self.map_plot.addLegend(labelTextColor=(0, 0, 0))
        self.legend_map.setBrush((255, 255, 255, 200))

        self.map_plots = {'master_path':self.map_plot.plot([], pen=pg.mkPen(plot_config['master_path_color'][:3], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['master_path_color'][:3], name=plot_config['master_label']+'-path'), 
                          'master_start':self.map_plot.plot([], symbolBrush=plot_config['master_path_color'][:3], symbolSize=17, symbol='star', symbolPen=plot_config['master_path_color'][:3], name=plot_config['master_label']+'-start'), 
                          'master_end':self.map_plot.plot([], symbolBrush=plot_config['master_path_color'][:3], symbolSize=17, symbol='+', symbolPen=plot_config['master_path_color'][:3], name=plot_config['master_label']+'-goal'), 
                          'obstacles_original':self.map_plot.plot([], pen=pg.mkPen(plot_config['obs_sta_org_color'], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['obs_sta_org_color'], name=plot_config['obs_org_legend']), 
                          'obstacle_bc':self.map_plot.plot([], pen=pg.mkPen(plot_config['obs_sta_org_color'], width=3, style=QtCore.Qt.DashLine), symbolBrush=(255, 255, 255), symbolSize=1, symbolPen=plot_config['obs_sta_org_color'], name=plot_config['obs_org_legend']), 
                        #   'obstacles_padded':self.map_plot.plot([], pen=pg.mkPen(plot_config['obs_sta_pad_color'], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['obs_sta_pad_color'], name=plot_config['obs_sta_pad_legend']), 
                          'closest_obstacles':self.map_plot.plot([], pen=pg.mkPen(plot_config['obs_sta_un_closest_color'], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['obs_sta_un_closest_color'], name=plot_config['obs_sta_un_closest_legend']), 
                          'traversed_path':self.map_plot.plot([], pen='b', symbolBrush=(255, 0, 0), symbolSize=5, symbolPen='g'), 
                          'planned_path':self.map_plot.plot([], pen=pg.mkPen(plot_config['planned_path_color'], style=QtCore.Qt.DashLine), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['planned_path_color'], name=plot_config['planned_path_label']), 
                          'planned_trajectory_master':self.map_plot.plot([], pen=pg.mkPen(plot_config['planned_trajectory_master_color']), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['planned_trajectory_master_color'], name=plot_config['planned_trajectory_master_legend']), 
                          'nodes':self.map_plot.plot([], pen=None, symbolBrush=(255, 255, 255), symbolSize=5, symbolPen=plot_config['node_color'], name=plot_config['node_label']), 
                          'boundry':self.map_plot.plot([], pen=pg.mkPen('k', width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen='k', name='Boundry'), 
                        #   'look_ahead':self.map_plot.plot([], pen=pg.mkPen(plot_config['look_ahead_color']), symbolBrush=(255, 255, 255), symbolSize=3, symbolPen=plot_config['look_ahead_color'], name=plot_config['look_ahead_legend']), 
                        #   'trajectory_master':self.map_plot.plot([], pen=pg.mkPen(plot_config['look_ahead_color']), symbolBrush=(255, 255, 255), symbolSize=3, symbolPen=plot_config['look_ahead_color'], name=plot_config['look_ahead_legend']+"-slave"), 
                          'line_vertices_master':self.map_plot.plot([], pen=pg.mkPen(plot_config['lines_master_color'], width=3), symbolBrush=(255, 255, 255), symbolSize=10, symbolPen=plot_config['lines_master_color'], name=plot_config['lines_legend_master']), 
                          'ref_point_master':self.map_plot.plot([], pen=None, symbolBrush=plot_config['master_ref_point_color'], symbolSize=10, symbolPen=plot_config['master_ref_point_color'], name=plot_config['master_ref_point_legend']), 
                        #   'search_sector':self.map_plot.plot([], pen=pg.mkPen(plot_config['search_sector_color'], width=3), symbolBrush=None, symbolSize=10, symbolPen=None, name=plot_config['search_sector_legend']), 
                          }
        
        if self.plot_config['plot_slave']:
            self.map_plots['slave_path'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['slave_path_color'][:3], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['slave_path_color'][:3], name=plot_config['slave_label']+'-path')
            self.map_plots['slave_start'] = self.map_plot.plot([], symbolBrush=plot_config['slave_path_color'][:3], symbolSize=17, symbol='star', symbolPen=plot_config['slave_path_color'][:3], name=plot_config['slave_label']+'-start')
            self.map_plots['slave_end'] = self.map_plot.plot([], symbolBrush=plot_config['slave_path_color'][:3], symbolSize=17, symbol='+', symbolPen=plot_config['slave_path_color'][:3], name=plot_config['slave_label']+'-goal')
            self.map_plots['planned_trajectory_slave'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['planned_trajectory_slave_color']), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['planned_trajectory_slave_color'], name=plot_config['planned_trajectory_slave_legend'])
            # self.map_plots['trajectory_slave'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['look_ahead_color']), symbolBrush=(255, 255, 255), symbolSize=3, symbolPen=plot_config['look_ahead_color'], name=plot_config['look_ahead_legend']+"-slave")
            self.map_plots['line_vertices_slave'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['lines_slave_color'], width=3), symbolBrush=(255, 255, 255), symbolSize=10, symbolPen=plot_config['lines_slave_color'], name=plot_config['lines_legend_slave'])
            self.map_plots['ref_point_slave'] = self.map_plot.plot([], pen=None, symbolBrush=plot_config['slave_ref_point_color'], symbolSize=10, symbolPen=plot_config['slave_ref_point_color'], name=plot_config['slave_ref_point_legend'])

        self.map_plots['object_collision'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['object_collision_color'][:1], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['object_collision_color'][:3], name='Cargo under the collision')
        self.map_plots['object_safe'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['object_no_collision_color'][:1], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['object_no_collision_color'][:3], name='safe Cargo')
        self.map_plots['cargo_bc'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['object_no_collision_color'][:1], width=2, style=QtCore.Qt.DashLine), symbolBrush=(255, 255, 255), symbolSize=1, symbolPen=plot_config['object_no_collision_color'][:3], name='Cargo')
        self.map_plots['object_collision_1'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['object_collision_color'][:1], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['object_collision_color'][:3], name='Cargo under the collision')
        self.map_plots['object_collision_2'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['object_collision_color'][:1], width=5), symbolBrush=(255, 255, 255), symbolSize=7,symbolPen=plot_config['object_collision_color'][:3])
        self.map_plots['object_collision_3'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['object_collision_color'][:1], width=5), symbolBrush=(255, 255, 255), symbolSize=7,symbolPen=plot_config['object_collision_color'][:3])
        self.map_plots['object_collision_4'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['object_collision_color'][:1], width=5), symbolBrush=(255, 255, 255), symbolSize=7,symbolPen=plot_config['object_collision_color'][:3])
        self.map_plots['object_collision_5'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['object_collision_color'][:1], width=5), symbolBrush=(255, 255, 255), symbolSize=7,symbolPen=plot_config['object_collision_color'][:3])
        
        self.map_plots['object_safe_1'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['object_no_collision_color'][:1], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['object_no_collision_color'][:3], name='safe Cargo')
        self.map_plots['object_safe_2'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['object_no_collision_color'][:1], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['object_no_collision_color'][:3])
        self.map_plots['object_safe_3'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['object_no_collision_color'][:1], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['object_no_collision_color'][:3])
        self.map_plots['object_safe_4'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['object_no_collision_color'][:1], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['object_no_collision_color'][:3])
        self.map_plots['object_safe_5'] = self.map_plot.plot([], pen=pg.mkPen(plot_config['object_no_collision_color'][:1], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['object_no_collision_color'][:3])
        
        # Set up data plots
        self.canvas_lin_vel = self.win_data.addPlot(title="Lin-Vel", row=0, col=0)
        self.canvas_lin_vel.setLabel('left', 'Vel', units='m/s')
        self.canvas_lin_vel.setLabel('bottom', 'Time', units='s')
        # self.legend_lin_vel = self.canvas_lin_vel.addLegend(labelTextColor=(0, 0, 0), labelTextSize='100pt')
        self.legend_lin_vel = self.canvas_lin_vel.addLegend(labelTextColor=(0, 0, 0))
        self.legend_lin_vel.setBrush((255, 255, 255, 200))

        self.canvas_lin_acc = self.win_data.addPlot(title="Lin-Acc", row=1, col=0)
        self.canvas_lin_acc.setLabel('left', 'Acc', units='m/s²')
        self.canvas_lin_acc.setLabel('bottom', 'Time', units='s')
        self.legend_lin_acc = self.canvas_lin_acc.addLegend(labelTextColor=(0, 0, 0))
        self.legend_lin_acc.setBrush((255, 255, 255, 200))

        self.canvas_ang_vel = self.win_data.addPlot(title="Ang-Vel", row=0, col=1)
        self.canvas_ang_vel.setLabel('left', 'Vel', units='rad/s')
        self.canvas_ang_vel.setLabel('bottom', 'Time', units='s')
        self.legend_ang_vel = self.canvas_ang_vel.addLegend(labelTextColor=(0, 0, 0))
        self.legend_ang_vel.setBrush((255, 255, 255, 200))

        self.canvas_ang_acc = self.win_data.addPlot(title="Ang-Acc", row=1, col=1)
        self.canvas_ang_acc.setLabel('left', 'Acc', units='rad/s²')
        self.canvas_ang_acc.setLabel('bottom', 'Time', units='s')
        self.legend_ang_acc = self.canvas_ang_acc.addLegend(labelTextColor=(0, 0, 0))
        self.legend_ang_acc.setBrush((255, 255, 255, 200))

        self.canvas_jerk = self.win_data.addPlot(title="Jerk", row=2, col=1)
        self.canvas_jerk.setLabel('left', 'Jerk', units='rad/s³ or m/s³')
        self.canvas_jerk.setLabel('bottom', 'Time', units='s')
        self.legend_jerk = self.canvas_jerk.addLegend(labelTextColor=(0, 0, 0))
        self.legend_jerk.setBrush((255, 255, 255, 200))

        self.canvas_constraint = self.win_data.addPlot(title="Constraint", row=2, col=0)
        self.canvas_constraint.setLabel('left', 'Deviation', units='m')
        self.canvas_constraint.setLabel('bottom', 'Time', units='s')
        self.legend_constraint = self.canvas_constraint.addLegend(labelTextColor=(0, 0, 0))
        self.legend_constraint.setBrush((255, 255, 255, 200))

        self.data_plots = {'master_lin_vel':{'data':self.canvas_lin_vel.plot([], pen=pg.mkPen(plot_config['master_vel_color'], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['master_vel_color'], name=plot_config['master_label']+'-lin-vel'), 
                                             'constraint':self.canvas_lin_vel.plot(x=np.array([0, 0, 0, 0], dtype=float), y=np.array([config.lin_vel_min, config.lin_vel_min, config.lin_vel_max, config.lin_vel_max], dtype=float), pen=pg.mkPen('k', width=3, style=QtCore.Qt.DashLine), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=None, name='Limits')}, 
                          
                          'master_lin_acc':{'data':self.canvas_lin_acc.plot([], pen=pg.mkPen(plot_config['master_vel_acc_color'], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['master_vel_acc_color'], name=plot_config['master_label']+'-lin-acc'), 
                                            'constraint':self.canvas_lin_acc.plot(x=np.array([0, 0, 0, 0], dtype=float), y=np.array([config.lin_acc_min, config.lin_acc_min, config.lin_acc_max, config.lin_acc_max], dtype=float), pen=pg.mkPen('k', width=3, style=QtCore.Qt.DashLine), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=None, name='Limits')}, 
                          
                          'master_lin_jerk':{'data':self.canvas_jerk.plot([], pen=pg.mkPen(plot_config['master_vel_jerk_color'], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['master_vel_jerk_color'], name=plot_config['master_label']+'-lin-jerk'), 
                                             'constraint':None}, 
                          
                          'master_ang_vel':{'data':self.canvas_ang_vel.plot([], pen=pg.mkPen(plot_config['master_ang_color'], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['master_ang_color'], name=plot_config['master_label']+'-ang-vel'), 
                                            'constraint':self.canvas_ang_vel.plot(x=np.array([0, 0, 0, 0], dtype=float), y=np.array([-config.ang_vel_max, -config.ang_vel_max, config.ang_vel_max, config.ang_vel_max], dtype=float), pen=pg.mkPen('k', width=3, style=QtCore.Qt.DashLine), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=None, name='Limits')}, 
                          
                          'master_ang_acc':{'data':self.canvas_ang_acc.plot([], pen=pg.mkPen(plot_config['master_ang_acc_color'], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['master_ang_acc_color'], name=plot_config['master_label']+'-ang-acc'), 
                                            'constraint':self.canvas_ang_acc.plot(x=np.array([0, 0, 0, 0], dtype=float), y=np.array([-config.ang_acc_max, -config.ang_acc_max, config.ang_acc_max, config.ang_acc_max], dtype=float), pen=pg.mkPen('k', width=3, style=QtCore.Qt.DashLine), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=None, name='Limits')}, 
                          
                          'master_ang_jerk':{'data':self.canvas_jerk.plot([], pen=pg.mkPen(plot_config['master_ang_jerk_color'], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['master_ang_jerk_color'], name=plot_config['master_label']+'-ang-jerk'), 
                                             'constraint':None},  
                          }

        if self.plot_config['plot_slave']:
            self.data_plots['slave_lin_vel']  = {'data':self.canvas_lin_vel.plot([], pen=pg.mkPen(plot_config['slave_vel_color'], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['slave_vel_color'], name=plot_config['slave_label']+'-lin-vel'), 
                                             'constraint':self.canvas_lin_vel.plot(x=np.array([0, 0, 0, 0], dtype=float), y=np.array([config.lin_vel_min, config.lin_vel_min, config.lin_vel_max, config.lin_vel_max], dtype=float), pen=pg.mkPen('k', width=3, style=QtCore.Qt.DashLine), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=None, name='Limits')} 
                          
            self.data_plots['slave_lin_acc']  = {'data':self.canvas_lin_acc.plot([], pen=pg.mkPen(plot_config['slave_vel_acc_color'], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['slave_vel_acc_color'], name=plot_config['slave_label']+'-lin-acc'), 
                                            'constraint':self.canvas_lin_acc.plot(x=np.array([0, 0, 0, 0], dtype=float), y=np.array([config.lin_acc_min, config.lin_acc_min, config.lin_acc_max, config.lin_acc_max], dtype=float), pen=pg.mkPen('k', width=3, style=QtCore.Qt.DashLine), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=None, name='Limits')} 
                          
            self.data_plots['slave_lin_jerk']  = {'data':self.canvas_jerk.plot([], pen=pg.mkPen(plot_config['slave_vel_jerk_color'], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['slave_vel_jerk_color'], name=plot_config['slave_label']+'-lin-jerk'), 
                                             'constraint':None} 
                          
            self.data_plots['slave_ang_vel']  = {'data':self.canvas_ang_vel.plot([], pen=pg.mkPen(plot_config['slave_ang_color'], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['slave_ang_color'], name=plot_config['slave_label']+'-ang-vel'), 
                                            'constraint':self.canvas_ang_vel.plot(x=np.array([0, 0, 0, 0], dtype=float), y=np.array([-config.ang_vel_max, -config.ang_vel_max, config.ang_vel_max, config.ang_vel_max], dtype=float), pen=pg.mkPen('k', width=3, style=QtCore.Qt.DashLine), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=None, name='Limits')}
                          
            self.data_plots['slave_ang_acc']  = {'data':self.canvas_ang_acc.plot([], pen=pg.mkPen(plot_config['slave_ang_acc_color'], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['slave_ang_acc_color'], name=plot_config['slave_label']+'-ang-acc'), 
                                            'constraint':self.canvas_ang_acc.plot(x=np.array([0, 0, 0, 0], dtype=float), y=np.array([-config.ang_acc_max, -config.ang_acc_max, config.ang_acc_max, config.ang_acc_max], dtype=float), pen=pg.mkPen('k', width=3, style=QtCore.Qt.DashLine), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=None, name='Limits')} 
                          
            self.data_plots['slave_ang_jerk']  = {'data':self.canvas_jerk.plot([], pen=pg.mkPen(plot_config['slave_ang_jerk_color'], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['slave_ang_jerk_color'], name=plot_config['slave_label']+'-ang-jerk'), 
                                             'constraint':None}
                          
            self.data_plots['constraint']  = {'data':self.canvas_constraint.plot([], pen=pg.mkPen(plot_config['master_path_color'], width=5), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=plot_config['master_path_color'], name=plot_config['constraint_legend']), 
                                        'constraint':self.canvas_constraint.plot(x=np.array([0, 0, 0, 0], dtype=float), y=np.array([config.constraint['distance_lb'], config.constraint['distance_lb'], config.constraint['distance_ub'], config.constraint['distance_ub']], dtype=float), pen=pg.mkPen('k', width=3, style=QtCore.Qt.DashLine), symbolBrush=(255, 255, 255), symbolSize=7, symbolPen=None, name='Limits')} 
                          


        # Set up costs plots
        self.cost_lin_vel = self.win_cost.addPlot(title="Lin", row=0, col=0)
        self.cost_lin_vel.setLabel('left', 'Cost')
        self.cost_lin_vel.setLabel('bottom', 'Time', units='s')
        self.legend_lin_vel = self.cost_lin_vel.addLegend(labelTextColor=(0, 0, 0))
        self.legend_lin_vel.setBrush((255, 255, 255, 200))

        self.cost_lin_acc = self.cost_lin_vel
        self.cost_lin_jerk = self.cost_lin_vel

        self.cost_ang_vel = self.win_cost.addPlot(title="Ang", row=0, col=1)
        self.cost_ang_vel.setLabel('left', 'Cost')
        self.cost_ang_vel.setLabel('bottom', 'Time', units='s')
        self.legend_ang_vel = self.cost_ang_vel.addLegend(labelTextColor=(0, 0, 0))
        self.legend_ang_vel.setBrush((255, 255, 255, 200))

        self.cost_ang_acc = self.cost_ang_vel
        self.cost_ang_jerk = self.cost_ang_vel

        self.cost_constraint = self.win_cost.addPlot(title="Constraint", row=1, col=0)
        self.cost_constraint.setLabel('left', 'Cost')
        self.cost_constraint.setLabel('bottom', 'Time', units='s')
        self.legend_constraint = self.cost_constraint.addLegend(labelTextColor=(0, 0, 0))
        self.legend_constraint.setBrush((255, 255, 255, 200))

        self.cost_checkpoint = self.win_cost.addPlot(title="Checkpoint", row=2, col=0)
        self.cost_checkpoint.setLabel('left', 'Cost')
        self.cost_checkpoint.setLabel('bottom', 'Time', units='s')
        self.legend_checkpoint = self.cost_checkpoint.addLegend(labelTextColor=(0, 0, 0))
        self.legend_checkpoint.setBrush((255, 255, 255, 200))

        self.cost_line_deviation = self.win_cost.addPlot(title="Line-deviation", row=2, col=1)
        self.cost_line_deviation.setLabel('left', 'Cost')
        self.cost_line_deviation.setLabel('bottom', 'Time', units='s')
        self.legend_line_deviation = self.cost_line_deviation.addLegend(labelTextColor=(0, 0, 0))
        self.legend_line_deviation.setBrush((255, 255, 255, 200))

        self.cost_obstacles = self.win_cost.addPlot(title="Obstacles", row=1, col=1)
        self.cost_obstacles.setLabel('left', 'Cost')
        self.cost_obstacles.setLabel('bottom', 'Time', units='s')
        self.legend_obstacles = self.cost_obstacles.addLegend(labelTextColor=(0, 0, 0))
        self.legend_obstacles.setBrush((255, 255, 255, 200))

        self.cost_plots = { 'cost_master_lin_vel':self.cost_lin_vel.plot([], pen=pg.mkPen(plot_config['cost_master_vel_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_master_vel_color'], name=plot_config['master_label']+'-lin-vel'), 
                            'cost_master_lin_acc':self.cost_lin_acc.plot([], pen=pg.mkPen(plot_config['cost_master_vel_acc_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_master_vel_acc_color'], name=plot_config['master_label']+'-lin-acc'), 
                            'cost_master_lin_jerk':self.cost_lin_jerk.plot([], pen=pg.mkPen(plot_config['cost_master_vel_jerk_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_master_vel_jerk_color'], name=plot_config['master_label']+'-lin-jerk'), 
                            'cost_master_ang_vel':self.cost_ang_vel.plot([], pen=pg.mkPen(plot_config['cost_master_ang_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_master_ang_color'], name=plot_config['master_label']+'-ang-vel'), 
                            'cost_master_ang_acc':self.cost_ang_acc.plot([], pen=pg.mkPen(plot_config['cost_master_ang_acc_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_master_ang_acc_color'], name=plot_config['master_label']+'-ang-acc'), 
                            'cost_master_ang_jerk':self.cost_ang_jerk.plot([], pen=pg.mkPen(plot_config['cost_master_ang_jerk_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_master_ang_jerk_color'], name=plot_config['master_label']+'-ang-jerk'), 
                            'cost_master_line_deviation':self.cost_line_deviation.plot([], pen=pg.mkPen(plot_config['master_path_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['master_path_color'], name=plot_config['master_label'] + '-' + plot_config['line_deviation_legend']), 
                            'cost_master_ref_points':self.cost_checkpoint.plot([], pen=pg.mkPen(plot_config['cost_master_ref_points_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_master_ref_points_color'], name=plot_config['cost_master_ref_points_legend']), 
                            'cost_master_static_obs':self.cost_obstacles.plot([], pen=pg.mkPen(plot_config['cost_master_static_obs_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_master_static_obs_color'], name=plot_config['cost_master_static_obs_legend']), 
                            'cost_master_dynamic_obs':self.cost_obstacles.plot([], pen=pg.mkPen(plot_config['cost_master_dynamic_obs_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_master_dynamic_obs_color'], name=plot_config['cost_master_dynamic_obs_legend']), 
                            'cost_master_vel_ref':self.cost_checkpoint.plot([], pen=pg.mkPen(plot_config['cost_master_vel_ref_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_master_vel_ref_color'], name=plot_config['cost_master_vel_ref_legend']), 

                            'cost_slave_lin_vel':self.cost_lin_vel.plot([], pen=pg.mkPen(plot_config['cost_slave_vel_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_slave_vel_color'], name=plot_config['slave_label']+'-lin-vel'), 
                            'cost_slave_lin_acc':self.cost_lin_acc.plot([], pen=pg.mkPen(plot_config['cost_slave_vel_acc_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_slave_vel_acc_color'], name=plot_config['slave_label']+'-lin-acc'), 
                            'cost_slave_lin_jerk':self.cost_lin_jerk.plot([], pen=pg.mkPen(plot_config['cost_slave_vel_jerk_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_slave_vel_jerk_color'], name=plot_config['slave_label']+'-lin-jerk'), 
                            'cost_slave_ang_vel':self.cost_ang_vel.plot([], pen=pg.mkPen(plot_config['cost_slave_ang_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_slave_ang_color'], name=plot_config['slave_label']+'-ang-vel'), 
                            'cost_slave_ang_acc':self.cost_ang_acc.plot([], pen=pg.mkPen(plot_config['cost_slave_ang_acc_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_slave_ang_acc_color'], name=plot_config['slave_label']+'-ang-acc'), 
                            'cost_slave_ang_jerk':self.cost_ang_jerk.plot([], pen=pg.mkPen(plot_config['cost_slave_ang_jerk_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_slave_ang_jerk_color'], name=plot_config['slave_label']+'-ang-jerk'), 
                            'cost_slave_ref_points':self.cost_checkpoint.plot([], pen=pg.mkPen(plot_config['cost_slave_ref_points_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_slave_ref_points_color'], name=plot_config['cost_slave_ref_points_legend']), 
                            'cost_slave_static_obs':self.cost_obstacles.plot([], pen=pg.mkPen(plot_config['cost_slave_static_obs_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_slave_static_obs_color'], name=plot_config['cost_slave_static_obs_legend']), 
                            'cost_slave_dynamic_obs':self.cost_obstacles.plot([], pen=pg.mkPen(plot_config['cost_slave_dynamic_obs_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_slave_dynamic_obs_color'], name=plot_config['cost_slave_dynamic_obs_legend']), 
                            'cost_slave_vel_ref':self.cost_checkpoint.plot([], pen=pg.mkPen(plot_config['cost_slave_vel_ref_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_slave_vel_ref_color'], name=plot_config['cost_slave_vel_ref_legend']), 
                            
                            'cost_constraint':self.cost_constraint.plot([], pen=pg.mkPen(plot_config['master_path_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['master_path_color'], name=plot_config['constraint_legend']), 
                            
                            'cost_circle':self.cost_obstacles.plot([], pen=pg.mkPen(plot_config['cost_circle_color'], width=5), symbolBrush=(255,255,255), symbolSize=7, symbolPen=plot_config['cost_circle_color'], name=plot_config['cost_circle_legend']),
                            'cost_future_circles':self.cost_obstacles.plot([], pen=pg.mkPen(plot_config['cost_circle_color'], width=5, style=QtCore.Qt.DashLine)),

                            'cost_future_master_lin_vel':self.cost_lin_vel.plot([], pen=pg.mkPen(plot_config['cost_master_vel_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_master_lin_acc':self.cost_lin_acc.plot([], pen=pg.mkPen(plot_config['cost_master_vel_acc_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_master_lin_jerk':self.cost_lin_jerk.plot([], pen=pg.mkPen(plot_config['cost_master_vel_jerk_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_master_ang_vel':self.cost_ang_vel.plot([], pen=pg.mkPen(plot_config['cost_master_ang_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_master_ang_acc':self.cost_ang_acc.plot([], pen=pg.mkPen(plot_config['cost_master_ang_acc_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_master_ang_jerk':self.cost_ang_jerk.plot([], pen=pg.mkPen(plot_config['cost_master_ang_jerk_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_master_line_deviation':self.cost_line_deviation.plot([], pen=pg.mkPen(plot_config['master_path_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_master_ref_points':self.cost_checkpoint.plot([], pen=pg.mkPen(plot_config['cost_master_ref_points_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_master_static_obs':self.cost_obstacles.plot([], pen=pg.mkPen(plot_config['cost_master_static_obs_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_master_dynamic_obs':self.cost_obstacles.plot([], pen=pg.mkPen(plot_config['cost_master_dynamic_obs_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_master_vel_ref':self.cost_checkpoint.plot([], pen=pg.mkPen(plot_config['cost_master_vel_ref_color'], width=5, style=QtCore.Qt.DashLine)), 

                            'cost_future_slave_lin_vel':self.cost_lin_vel.plot([], pen=pg.mkPen(plot_config['cost_slave_vel_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_slave_lin_acc':self.cost_lin_acc.plot([], pen=pg.mkPen(plot_config['cost_slave_vel_acc_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_slave_lin_jerk':self.cost_lin_jerk.plot([], pen=pg.mkPen(plot_config['cost_slave_vel_jerk_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_slave_ang_vel':self.cost_ang_vel.plot([], pen=pg.mkPen(plot_config['cost_slave_ang_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_slave_ang_acc':self.cost_ang_acc.plot([], pen=pg.mkPen(plot_config['cost_slave_ang_acc_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_slave_ang_jerk':self.cost_ang_jerk.plot([], pen=pg.mkPen(plot_config['cost_slave_ang_jerk_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_slave_ref_points':self.cost_checkpoint.plot([], pen=pg.mkPen(plot_config['cost_slave_ref_points_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_slave_static_obs':self.cost_obstacles.plot([], pen=pg.mkPen(plot_config['cost_slave_static_obs_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_slave_dynamic_obs':self.cost_obstacles.plot([], pen=pg.mkPen(plot_config['cost_slave_dynamic_obs_color'], width=5, style=QtCore.Qt.DashLine)), 
                            'cost_future_slave_vel_ref':self.cost_checkpoint.plot([], pen=pg.mkPen(plot_config['cost_slave_vel_ref_color'], width=5, style=QtCore.Qt.DashLine)), 
                            
                            
                            'cost_future_constraint':self.cost_constraint.plot([], pen=pg.mkPen(plot_config['master_path_color'], width=5, style=QtCore.Qt.DashLine)), 
                            }

    def show(self):
        self.win_map.show()
        self.win_map.resize(self.width, self.height) 
        self.win_map.raise_()

        self.win_data.show()
        self.win_data.resize(self.width, self.height) 
        self.win_data.raise_()

        self.win_cost.show()
        self.win_cost.resize(self.width, self.height) 
        self.win_cost.raise_()

    def update_map(self, key, data):
        if key == 'obstacles_original' or key == 'look_ahead' or key == 'closest_obstacles' or key == 'obstacles_padded':
            connect = []
            x = []
            y = []
            for d in data:
                try:
                    x_, y_ = tuple(np.array(d).T)
                except:
                    print(f"{self.print_name}: data put on plot_queues has to be either [x, y] or [[[x0, y0], [x1, y1]], [[x0, y0], [x1, y1]] ]")
                    return
                x += list(x_) + [x_[0]]
                y += list(y_) + [y_[0]]
                connect += [1]*(x_.shape[0]) + [0]
            self.map_plots[key].setData(x=x, y=y, connect=np.array(connect))
        
        elif key == 'obstacle_bc' or key == 'cargo_bc':
            connect = []
            x = []
            y = []
            r_extension = 0.7247

            for d in data:  #data is the list of circle parameters,  [array([x, y]) radius]
                
                xy_origin, r = tuple(np.array(d).T)

                o_x = xy_origin[0]
                o_y = xy_origin[1]
                n = 200
                list_points = [(math.cos(2*math.pi/n*x)*r + o_x,math.sin(2*math.pi/n*x)*r+o_y) for x in range(0,n+1)]
                try:
                    x_, y_ = tuple(np.array(list_points).T)

                except:
                    print(f"{self.print_name}: some exception: data put on plot_queues has to be either [x, y] or [[[x0, y0], [x1, y1]], [[x0, y0], [x1, y1]] ]")
                    return
                x += list(x_) + [x_[0]]
                y += list(y_) + [y_[0]]
                connect += [1]*(x_.shape[0]) + [0]
            self.map_plots[key].setData(x=x, y=y, connect=np.array(connect))
        
        else:
            try:
                x, y = tuple(data)
            except ValueError:
                print(f"{self.print_name}: data put on plot_queues has to be either [x, y] or [[[x0, y0], [x1, y1]], [[x0, y0], [x1, y1]] ]")
                return
            
            self.map_plots[key].setData(x, y)

    def update_data(self, key, data):
        try:
            x, y = tuple(data)
        except ValueError:
            print(f"{self.print_name}: data put on costs queues has to be either [x, y] or [[[x0, y0], [x1, y1]], [[x0, y0], [x1, y1]] ]")
        
        self.data_plots[key]['data'].setData(x, y)

        #Update constraint plot
        if self.data_plots[key]['constraint'] is not None:
            x_constraint = self.data_plots[key]['constraint'].xData
            x_constraint[[1, 3]] = x.max()
            y_constraint = self.data_plots[key]['constraint'].yData
            self.data_plots[key]['constraint'].setData(x_constraint, y_constraint, connect='pairs')
        
    def update_cost(self, key, data):
        try:
            x, y = tuple(data)
        except ValueError:
            print(f"{self.print_name}: data put on costs queues has to be either [x, y]")

        self.cost_plots[key].setData(x, y)
    
    def save_plots(self):
        leader_follower_str = '/leader_'
        if self.plot_config['plot_slave']:
            leader_follower_str += 'follower_'

        exporter_map = pg.exporters.SVGExporter(self.map_plot)
        map_path = self.aut_test_config['path']+'/maps'+ leader_follower_str
        
        map_path += 'graph'+str(self.aut_test_config['graph'])+'_map'+".svg"
        #should be 'aut_testing/test_name/maps/graph1.png'
        exporter_map.export(map_path)

        exporter_data = self.win_data.grab()
        data_path = self.aut_test_config['path']+'/data'+ leader_follower_str + 'graph'  + str(self.aut_test_config['graph'])+'_data'+self.aut_test_config['file_extension']
        exporter_data.save(data_path)

        exporter_cost = self.win_cost.grab()
        costs_path = self.aut_test_config['path']+'/costs'+'/graph'+ leader_follower_str +str(self.aut_test_config['graph'])+'_costs'+self.aut_test_config['file_extension']
        exporter_cost.save(costs_path)

    def check_plot_queues(self):
        if self.aut_test_config is not None and self.aut_test_config['plot_event'].is_set():
            print("I'm gonna save!!!")
            self.save_plots()
            self.aut_test_config['plot_event'].clear()

        data = None
        # Update map plot
        for key in self.plot_queues_map:
            try:
                data = self.plot_queues_map[key].get(block=False)
                # print("Got data")

            except Empty: 
                # print("Got no data")
                continue
            
            self.update_map(key, data)

        # Update data plot
        for key in self.plot_queues_data:
            try:
                data = self.plot_queues_data[key].get(block=False)
                # print("Got data")
        
            except Empty: 
                # print("Got no data")
                continue
            
            self.update_data(key, data)

        # Update cost plot
        for key in self.plot_queues_cost:
            try:
                data = self.plot_queues_cost[key].get(block=False)
                # print("Got data")
        
            except Empty: 
                # print("Got no data")
                continue
            
            self.update_cost(key, data)
            


    