import math
import os, sys, glob
from time import time

import numpy as np
import pandas as pd
import cv2

try:
    from utils.utils_sl import *
except:
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
    from utils.utils_sl import *


'''
    1   Track ID. All rows with the same ID belong to the same path.
    2   xmin. The top left x-coordinate of the bounding box.
    3   ymin. The top left y-coordinate of the bounding box.
    4   xmax. The bottom right x-coordinate of the bounding box.
    5   ymax. The bottom right y-coordinate of the bounding box.
    6   frame. The frame that this annotation represents.
    7   lost. If 1, the annotation is outside of the view screen.
    8   occluded. If 1, the annotation is occluded.
    9   generated. If 1, the annotation was automatically interpolated.
    10  label. The label for this annotation, enclosed in quotation marks.
'''

font = cv2.FONT_HERSHEY_COMPLEX_SMALL

class ReadVideo():

    def __init__(self, path, scenario_idx=0, video_idx=0, verbose=True):
        
        self.scenario_list = os.listdir(path+'videos/') # all scenarios

        self.sce_op = self.scenario_list[scenario_idx]  # selected scenario
        path_op = path + 'videos/' + self.sce_op
        path_an = path + 'annotations/' + self.sce_op
        files_v = glob.glob(path_op+'/*/*.*')           # all video files
        files_a = glob.glob(path_an+'/*/*.txt')         # all annotation files

        self.video_path = files_v[video_idx]
        self.cap = cv2.VideoCapture(files_v[video_idx])
        self.__fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.__width  = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.__height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        anno = files_a[video_idx]
        self.df_data = pd.read_csv(anno, sep=' ', header=None,
                        names=['ID','xmin','ymin','xmax','ymax',
                        'frame','lost','occluded','generated','label'])

        self._idx_frame = -1
        self._vb = verbose
        if self._vb:
            self.intro()

    def intro(self):
        print('='*30)
        print(f'All available scenarios in SDD: {self.scenario_list}.')
        print(f'Selected scene: [{self.sce_op}].')
        print(f'Video path: [{self.video_path}].')
        print(f'Video info: FPS-{self.__fps}, Size-({self.__width}, {self.__height})')
        print('='*30)

    def info(self, keyword):
        info_dict = {'fps':self.__fps, 'FPS':self.__fps, 'width':self.__width, 'height':self.__height}
        return info_dict[keyword]

    def resize_frame(self,frame,size:tuple):
        return cv2.resize(frame, size, fx=0,fy=0, interpolation=cv2.INTER_CUBIC)

    def reset_frame_idx(self):
        self._idx_frame = -1
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    def read_frame_clean(self):
        ret, frame = self.cap.read()
        assert(ret),('No frame available from the source.')
        df_frame = self.df_data[self.df_data['frame']==self._idx_frame]
        self._idx_frame += 1
        return frame, df_frame

    def read_frame_annotated(self, show_lost=False):
        frame, df_frame = self.read_frame_clean()
        for frm in range(df_frame.shape[0]):
            rec = df_frame.iloc[frm,:]
            [xmin, ymin, xmax, ymax] = rec.loc[['xmin','ymin','xmax','ymax']]
            center = tuple([int((xmin+xmax)/2), int((ymin+ymax)/2)])
            if rec['lost']:
                if show_lost:
                    cv2.circle(frame, center, 2, (0,0,0), 3)
            else:
                if rec['occluded']:
                    cv2.circle(frame, center, int((xmax-xmin)/2), (0,0,255), 2)
                cv2.rectangle(frame, tuple([xmin,ymin]), tuple([xmax,ymax]), (0,255,0), 1)
                text_with_backgroud(frame, rec['label'], tuple([xmin, ymin]), color_text=(0,0,0), color_bg=(0,255,0))

            # legend info
            cv2.circle(frame, (5,15), 5, (0,0,255), 2)
            cv2.rectangle(frame, (1,30), (10,40), (0,255,0), 1)                     
            cv2.putText(frame, ': occluded', (12,20), font, 1, (0,0,255))
            cv2.putText(frame, ': detected', (12,40), font, 1, (0,255,0))
        return frame, df_frame

    def play_video(self, mode=0):
        if mode==0:
            self.play_video_clean()
        elif mode==1:
            self.play_video_annotated()
        else:
            raise ModuleNotFoundError()

    def play_video_clean(self, new_size=None):
        end = 0
        while(1):
            try:
                frame, _ = self.read_frame_clean()
            except:
                break
            if new_size is not None:
                frame = self.resize_frame(frame, new_size)
            cv2.putText(frame, f'FPS: {round(1/(time()-end),1)}', (0,60), font, 1, (255,0,0))
            cv2.imshow(f'SDD-{self.sce_op}',frame)
            end = time()
            if cv2.waitKey(int(1000/self.__fps)) & 0xFF==ord('q'): 
                break
        self.cap.release()
        cv2.destroyAllWindows()

    def play_video_annotated(self, new_size=None, show_lost=False):
        end = 0  # for FPS control
        end2 = 0 # for FPS control
        cnt = 21 # for FPS control
        while(1):
            delay = 1  # for FPS control
            cnt += 1   # for FPS control
            try:
                frame, _ = self.read_frame_annotated(show_lost=show_lost)
            except:
                break
            if new_size is not None:
                frame = self.resize_frame(frame, new_size)
            real_fps = 1/(time()-end)  # for FPS control
            proc_fps = 1/(time()-end2) # for FPS control
            if cnt > 20:
                cnt = 0
                show_fps = round(real_fps,1)
            cv2.putText(frame, f'FPS: {show_fps}', (0,60), font, 1, (255,0,0))
            self.real_world_scale(frame=frame)
            cv2.imshow(f'SDD-{self.sce_op}',frame)
            end = time()  # for FPS control
            if 1000/self.__fps > 1000/proc_fps:
                delay += int(1000/self.__fps - 1000/proc_fps)
            # print(f'Want:{1000/self.__fps}ms/fr  Real:{1000/real_fps}ms/fr  Delay:{delay}ms/fr')
            if cv2.waitKey(delay) & 0xFF==ord('q'): 
                break
            end2 = time() # for FPS control
        self.cap.release()
        cv2.destroyAllWindows()

    def play_video_extra(self, extra, new_size=None, show_lost=False):
        # extra: {'func_evolve':func1, 'func_draw':func2, 'ts':[second]}
        end = 0  # for FPS control
        end2 = 0 # for FPS control
        cnt = 21 # for FPS control
        cnt_extra = 0 # for FPS compatibility
        while(1):
            delay = 1  # for FPS control
            cnt += 1   # for FPS control
            cnt_extra += 1
            try:
                frame, _ = self.read_frame_annotated(show_lost=show_lost)
            except:
                break

            if cnt_extra >= 3:
                cnt_extra = 0
                extra['func_evolve']()
                extra['func_draw'](frame)
            else:
                extra['func_draw'](frame)

            if new_size is not None:
                frame = self.resize_frame(frame, new_size)
            real_fps = 1/(time()-end)  # for FPS control
            proc_fps = 1/(time()-end2) # for FPS control
            if cnt > 20:
                cnt = 0
                show_fps = round(real_fps,1)
            cv2.putText(frame, f'FPS: {show_fps}', (0,60), font, 1, (255,0,0))
            self.real_world_scale(frame=frame)
            cv2.imshow(f'SDD-{self.sce_op}',frame)
            end = time()  # for FPS control
            if 1000/self.__fps > 1000/proc_fps:
                delay += int(1000/self.__fps - 1000/proc_fps)
            # print(f'Want:{1000/self.__fps}ms/fr  Real:{1000/real_fps}ms/fr  Delay:{delay}ms/fr')
            if cv2.waitKey(delay) & 0xFF==ord('q'): 
                break
            end2 = time() # for FPS control
        self.cap.release()
        cv2.destroyAllWindows()

    def real_world_scale(self, px2m_scale=0.036, frame=None):
        # px2m_scale means 1 pixel is such many meters.
        m_in_px = int(1/px2m_scale)
        if frame is not None:
            cv2.rectangle(frame, (int(self.__width/2),int(self.__height/2)), (int(self.__width/2)+m_in_px, int(self.__height/2)+m_in_px), (255,0,0), 1)
            cv2.putText(frame, '1m box', (int(self.__width/2),int(self.__height/2-2)), font, 1, (255,0,0))
        return m_in_px



if __name__ == '__main__':

    print(cv2.__version__)
    path = '/media/ze/Elements/User/Data/SDD/'
    reader = ReadVideo(path)
    reader.play_video(mode=1)