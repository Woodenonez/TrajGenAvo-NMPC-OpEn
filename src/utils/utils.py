import sys
import json

import cv2
'''
A stardard json file storing "predictions" should be:
{'info':[t1,x,y], 'pred_T1':[[a1,x1,y1,sx1,sy1], ..., [am,xm,ym,sxm,sym]], 'pred_T2':..., ...}
{'info':[t2,x,y], 'pred_T1':[[a1,x1,y1,sx1,sy1], ..., [am,xm,ym,sxm,sym]], 'pred_T2':..., ...} ...
One file for one object. Each row is a pred_t.

A stardard json file storing "trajectories" should be:
{'type':type, 'traj_x':[x1,x2,x3,...], 'traj_y':[y1,y2,y3,...]}
{'type':type, 'traj_x':[x1,x2,x3,...], 'traj_y':[y1,y2,y3,...]} ...
One file for multiple trajectories. Each row is a trajectory.
'''
def save_obj_as_json(obj_list, json_file_path):
    # pred_list: [pred_t1, pred_t2, ...]
    # traj_list: [traj1,   traj2,   ...]
    with open(json_file_path,'w+') as jf:
        for obj in obj_list:
            json.dump(obj, jf)
            jf.write('\n')

def read_obj_from_json(json_file): # return a list of dict
    obj_list = []
    with open(json_file,'r+') as jf:
        for obj in jf:
            try:
                obj_list.append(json.loads(obj))
            except: pass
    return obj_list

'''
GUI
'''
def text_with_backgroud(img, text, org, font=cv2.FONT_HERSHEY_COMPLEX_SMALL, scale=1, color_text=(0,0,0), color_bg=(255,255,255)):
    (txt_w, txt_h) = cv2.getTextSize(text, font, fontScale=scale, thickness=1)[0]
    cv2.rectangle(img, tuple([org[0], org[1]-txt_h-3]), tuple([org[0]+txt_w, org[1]]), color_bg, cv2.FILLED)
    cv2.putText(img, text, tuple([org[0],org[1]-3]), font, scale, color_text)


