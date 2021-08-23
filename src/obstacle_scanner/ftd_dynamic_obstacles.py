import os, sys, math
import json
import numpy as np

'''
File info:
    Name    - [ftd_dynamic_obstacles]
    Author  - [Ze]
    Date    - [Jan. 01, 2021] -> [Aug. 20, 2021]
    Exe     - [Yes]
File description:
    Load the dynamic obstacles and their future predictions from json files.
File content:
    ObstacleScanner <class> - Offering information and MPC-format list of dynamic obstacles.
Comments:
    This code is a disaster but works, so...
'''

class SimpleConfig():
    def __init__(self):
        self.ts = 0.2
        self.vehicle_width  = 0.5
        self.vehicle_margin = 0.25

class ObstacleScanner():
    '''
    Description:
        Generate/read the information of dynamic obstacles, and form it into the correct format for MPC.
    Attributes:
        num_obstacles <int> - The number of (active) dynamic obstacles.
    Functions
        get_obstacle_info      <get> - Get the position, orientation, and shape information of a selected (idx) obstacle.
        get_full_obstacle_list <get> - Form the list for MPC problem.
    Comments:
        Other attributes and functions are just for testing and specific in thie file.
    '''
    def __init__(self):
        self.num_obstacles = 1 # fixed here
        self.config = SimpleConfig() # helper
        self.start_time = 0 * self.config.ts
        self.start_offset = 2

        json_path = '/home/ze/Documents/Code/Python_Code/TrajGenAvo_NMPC_OpEn/src/obstacle_scanner/pred_result/0_4'
        self.read_obj_from_json(json_path)

    def get_obstacle_info(self, idx, current_time, key): # the start time of the first prediction
        obs_dict = self.get_obs_dict(current_time)
        if obs_dict is None:
            obs_dict = {'position':(0,0), 'pos':(0,0), 'radius':(0,0), 'axis':(0,0), 'heading':0, 'angle':0}
        else:
            pos = (obs_dict['info'][1],obs_dict['info'][2])
            rx = 0.2 # + self.config.vehicle_width/2 + self.config.vehicle_margin
            ry = 0.2 # + self.config.vehicle_width/2 + self.config.vehicle_margin
            obs_dict = {'position':pos, 'pos':pos, 'radius':(rx,ry), 'axis':(rx,ry), 'heading':0, 'angle':0}
        if key==-1:
            return obs_dict
        return obs_dict[key]

    def get_full_obstacle_list(self, current_time, horizon, ts=0):
        num_compo = 2
        ts = self.config.ts
        obs_dict = self.get_obs_dict(current_time) # pred
        if obs_dict is None:
            return []

        obstacles_list = []
        for n in range(num_compo):
            obs_list = []
            for T in range(horizon):
                try:
                    obs_pred = obs_dict[list(obs_dict)[T+1]][n]
                except:
                    obs_pred = obs_dict[list(obs_dict)[T+1]][0]
                x  = obs_pred[1]
                y  = obs_pred[2]
                rx = obs_pred[3]
                ry = obs_pred[4]
                rx += self.config.vehicle_width/2 + self.config.vehicle_margin
                ry += self.config.vehicle_width/2 + self.config.vehicle_margin
                obs_T = (x, y, rx, ry, 0)
                obs_list.append(obs_T)
            obstacles_list.append(obs_list)
        return obstacles_list # x, y, rx, ry, theta for t in horizon

    # Just for FTD
    def read_obj_from_json(self, json_path):
        '''
        Description:
            Read the dynamic obstacles and predictions from json file.
            Each file is an object!!!
        Arguments:
            json_path <str> - The path of the json file.
        Return:
            json_obj_list <list of dicts> - Each dictionary contains all info of the target at a time instant.
        Comments:
            The format of the dictionary is
                {'info':[t1,x,y], 'pred_T1':[[a1,x1,y1,sx1,sy1], ..., [am,xm,ym,sxm,sym]], 'pred_T2':..., ...}
                where 'm' is the number of components/futures.
        '''
        self.json_obj_list = []
        with open(json_path,'r+') as jf:
            for obj in jf:
                try:
                    self.json_obj_list.append(json.loads(obj))
                except: pass
        return self.json_obj_list

    def get_obs_dict(self, current_time):
        if current_time >= self.start_time:
            time_step = int((current_time-self.start_time)/self.config.ts)
            if (time_step+self.start_offset) <= len(list(self.json_obj_list))-1:
                obs_dict = self.json_obj_list[time_step+self.start_offset]
                return obs_dict # pred
        return None
        

    
if __name__ == '__main__':

    # from pathlib import Path
    # sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
    # from utils.config import Configurator

    # yaml_fp = os.path.join(str(Path(__file__).parent.parent.parent), 'configs/default.yaml')
    # configurator = Configurator(yaml_fp)
    # config = configurator.configurate()

    reader = ObstacleScanner()

    info = reader.get_obstacle_info(0, 0, -1, start_time=0)
    print(info)
    reader.get_full_obstacle_list(current_time=0, horizon=20, ts=0.2, start_time=0)

