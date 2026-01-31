#!usr/bin/env python3

##################################
# Imports
##################################
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup


# Custom Services
from exam_interfaces.srv import StartNavigation, MoveHead, ManipulatorCommand

# Python Libraries
import os
import csv
import time
from threading import Thread


##################################
# Coordinator Node
##################################
class CoordinatorTask2(Node):
    def __init__(self):
        super().__init__('coordinator_task2')

        ##################################
        # Callback Group
        ##################################
        self.callback_group = ReentrantCallbackGroup()

        ##################################
        # Class Parameters
        ##################################
        ws_src_path = os.path.join(os.path.expanduser('~'), 'tiago_gh_ws/src/tiago_exam')
        map_path_dir = os.path.join(ws_src_path, 'maps')
        data_path_dir = os.path.join(ws_src_path, 'data')
        
        # Declare parameters fallback
        self.map_path = self.declare_parameter('map_path', os.path.join(map_path_dir, 'group29_env_map.yaml')).get_parameter_value().string_value
        self.marker_file_path = self.declare_parameter('marker_file_path', os.path.join(data_path_dir, 'marker_locations.csv')).get_parameter_value().string_value

        ##################################
        # State Variables
        ##################################

        ##################################
        # Cubes Variables
        ##################################

        ##################################
        # Service Clients
        ##################################
        self.nav_client = self.create_client(StartNavigation, 'start_nav_srv', callback_group=self.callback_group)
        self.head_client = self.create_client(MoveHead, 'move_head_srv', callback_group=self.callback_group)

        ##################################
        # Subscriber
        ##################################

        ##################################
        # Publisher
        ##################################