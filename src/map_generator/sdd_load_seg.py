import os, sys, glob
import json
import pathlib

import numpy as np

import cv2
import PIL.Image
from matplotlib import pyplot as plt

class GenSegmentation(): # use labelme: labelme_json_to_dataset [*.json] -o [dir]
    def __init__(self, label_dir, verbose=True):
        pass


class ReadSegmentation():

    def __init__(self, label_dir, verbose=True):
        self.vb = verbose
        self.label_dir = label_dir
        self.label = np.asarray(PIL.Image.open(label_dir+'label.png'))
        self.label_gray = self.label/int(np.max(self.label))
        self.num_class = int(np.max(self.label)) + 1

        with open(label_dir+'label_names.txt', 'r') as f:
            lines = f.readlines()
        self.class_list = [l[:-1] for l in lines]
        self.label_list = []

        if self.vb:
            self.intro()

    def intro(self):
        print('='*30)
        print(f'Classes: {self.class_list}.')
        print(f'Image Size: ({self.label.shape[1]}, {self.label.shape[0]})')
        print('='*30)

    def gen_label(self):
        for i in range(self.num_class):
            blank = np.zeros(self.label.shape)
            blank[self.label==i] = 1
            self.label_list.append(blank)
        return self.label_list

    def plot_labels(self, label_index=-1):
        self.gen_label()
        assert(np.max(label_index)<self.num_class),('Wrong index!')
        if (label_index is not list) & (label_index != -1):
            label_index = [label_index]
        if label_index is list:
            select_labels  = [self.label_list[idx] for idx in label_index]
            select_classes = [self.class_list[idx] for idx in label_index]
        elif label_index == -1:
            select_labels  = self.label_list
            select_classes = self.class_list

        for i in range(len(select_labels)):
            plt.subplot(1,self.num_class+1, i+1), plt.imshow(select_labels[i], cmap='gray')
            plt.title(select_classes[i])
        plt.subplot(1,self.num_class+1, self.num_class+1), plt.imshow(self.label_gray)
        plt.title('[Seg]')
        plt.show()

    def gen_convex_hulls(self, label_index, squeeze=False,  plot_result=False):
        self.gen_label()
        img_gray = (self.label_list[label_index]*255).astype(np.uint8)
        img_color = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
        contours, _  = cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        num_area = len(contours)
        if self.vb:
            print(f'{num_area} areas/contours detected.')
        hull_list = []
        for i in range(num_area):
            '''
            https://docs.opencv.org/4.5.1/d3/dc0/group__imgproc__shape.html#ga014b28e56cb8854c0de4a211cb2be656
            Jack Sklansky. Finding the convex hull of a simple polygon. Pattern Recognition Letters, 1(2):79–83, 1982.
            '''
            hull_list.append(cv2.convexHull(contours[i]))

        if plot_result:
            for i in range(num_area):
                cv2.drawContours(img_color, contours, i, (0,100,0), 5)
                cv2.drawContours(img_color, hull_list, i, (0,100,255), 2)
            cv2.imshow('Contours and convex hulls.',img_color)
            cv2.waitKey(0)

        if squeeze:
            hull_list = [hull.squeeze(axis=1) for hull in hull_list]

        return hull_list

if __name__ == '__main__':

    label_dir = os.path.join(str(pathlib.Path(__file__).parent), 'SDD_seg/Bookstore/')
    reader = ReadSegmentation(label_dir)
    reader.plot_labels(-1)