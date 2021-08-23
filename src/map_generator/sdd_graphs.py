import os, sys, math
import pathlib
from time import time

import cv2
import numpy as np
import matplotlib.pyplot as plt

from sdd_load_video import ReadVideo
from sdd_load_seg import ReadSegmentation
# from robot_dynamics import MotionModel

font = cv2.FONT_HERSHEY_COMPLEX_SMALL

class Graph:
    def __init__(self, video_dir, label_dir):
        self.video_dir = video_dir
        self.label_dir = label_dir
        self.load()
        self.boundary_coordinates = self.return_boundary_coordinates()
        self.obstacle_list = self.return_obstacle_list()
        self.start = (1,1, math.radians(0))
        self.end = (1000,1000, math.radians(0))

    def load(self):
        self.v_reader = ReadVideo(self.video_dir)
        self.m_in_px = self.v_reader.real_world_scale(0.036, None)

        self.s_reader = ReadSegmentation(self.label_dir)
        self.hulls = self.s_reader.gen_convex_hulls(label_index=self.s_reader.class_list.index('Stop'))

    def return_boundary_coordinates(self):
        boundary_coordinates = [(0.0, 0.0), 
            (self.v_reader.info('width'), 0.0), 
            (self.v_reader.info('width'), self.v_reader.info('height')),
            (0.0, self.v_reader.info('height'))]
        return boundary_coordinates

    def return_obstacle_list(self):
        obstacle_list = []
        for hull in self.hulls:
            hull.squeeze(axis=1)
            obstacle = []
            for i in range(hull.shape[0]):
                obstacle.append(tuple(hull[i,:]))
            obstacle_list.append(obstacle)
        return obstacle_list


    def func_evolve(self):
        pass
        # robot.one_step(u, border=[0,0,v_reader.info('width'),v_reader.info('height')])
    def func_draw(self, frame):
        # shape = robot.robot_shape(scale=m_in_px, cv=True)
        # cv2.drawContours(frame, [shape], 0, robot.color_in_cv(), 2)
        cv2.drawContours(frame, self.hulls, -1, (0,100,255), 2)
    def play_video_with_segmentation(self):
        extra = {'func_evolve':self.func_evolve, 'func_draw':self.func_draw, 'ts':0.1}
        self.v_reader.play_video_extra(extra)


if __name__ == '__main__':

    video_dir = '/media/ze/Elements/User/Data/SDD/'
    label_dir = os.path.join(str(pathlib.Path(__file__).parent), 'SDD_seg/Bookstore/')
    graph = Graph(video_dir, label_dir)
    graph.play_video_with_segmentation()
    # print(graph.hulls[0].shape)
    # plt.plot()


