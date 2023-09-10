import torch as T
import torch.nn as nn
from torchvision import transforms, models
from torchvision.transforms import InterpolationMode
from PIL import Image, ImageGrab
import numpy as np
import cv2
from time import time, sleep
import configparser

cf = configparser.ConfigParser()
cf.read('./vision/config.ini')
cf_pos = (cf.getint('screen', 'L'), cf.getint('screen', 'U'), 
          cf.getint('screen', 'R'), cf.getint('screen', 'D'))
row_ign = cf.get('screen', 'hor_ign').split('/')
row_ign = float(row_ign[0])/float(row_ign[1])
col_ign = cf.get('screen', 'ver_ign').split('/')
col_ign = float(col_ign[0])/float(col_ign[1])

mask = Image.open(cf.get('predictor_mask', 'tip'))
mask = transforms.ToTensor()(mask)
mask = (mask-0.1)*20
mask = mask.reshape((1,1,39,39))

device = 'cuda' if T.cuda.is_available() and cf.getint('basic', 'cuda') else 'cpu'
model_rcv = models.shufflenet_v2_x1_5()
model_rcv.conv1[0] = nn.Conv2d(1, 24, kernel_size=(3, 3), stride=(2, 2), padding=(1, 1), bias=False)
model_rcv.fc = nn.Linear(1024, 1, bias=False)
model_rcv.load_state_dict(T.load(cf.get('models', 'recover'), map_location=T.device('cpu')))
model_rcv.eval()
model_rcv.to(device)
model_pick = models.shufflenet_v2_x1_5()
model_pick.conv1[0] = nn.Conv2d(1, 24, kernel_size=(3, 3), stride=(2, 2), padding=(1, 1), bias=False)
model_pick.fc = nn.Linear(1024, 1, bias=False)
model_pick.load_state_dict(T.load(cf.get('models', 'pick'), map_location=T.device('cpu')))
model_pick.eval()
model_pick.to(device)

model_place = models.shufflenet_v2_x1_5()
model_place.conv1[0] = nn.Conv2d(1, 24, kernel_size=(3, 3), stride=(2, 2), padding=(1, 1), bias=False)
model_place.fc = nn.Linear(1024, 1, bias=False)
model_place.load_state_dict(T.load(cf.get('models', 'place'), map_location=T.device('cpu')))
model_place.eval()
model_place.to(device)

crop_len = 140


def set_crop_len(new_len):
    global crop_len
    crop_len = new_len

def get_crop_len():
    global crop_len
    return crop_len

def is_on_tip(img):

    '''
        place
        判断岛是否在针尖上
        input：img_after-img_before
        return: 判断结果，置信度
    '''
    img = transforms.Grayscale(num_output_channels=1)(img)
    src= transforms.Resize((224, 224), interpolation=InterpolationMode.BILINEAR)(img)
    src = transforms.ToTensor()(src).unsqueeze(0).to(device)
    pre = T.sigmoid(model_place(src)).cpu().item()
    print(pre)
    if cf.getint('basic', 'cache'):
        if pre > 0.5:
            img.save('cache/Place/1/%d_place_%.3f.png' % (int(time()), pre))
        else:
            img.save('cache/Place/0/%d_place_%.3f.png' % (int(time()), pre))
    return round(pre), pre

def is_on_tip2(img):
    '''
        pick
        判断岛是否在针尖上
        input：img_after-img_before
        return: 判断结果，置信度
    '''
    img = transforms.Grayscale(num_output_channels=1)(img)
    src= transforms.Resize((224, 224), interpolation=InterpolationMode.BILINEAR)(img)
    src = transforms.ToTensor()(src).unsqueeze(0).to(device)
    pre = T.sigmoid(model_pick(src)).cpu().item()
    print (pre)
    if cf.getint('basic', 'cache'):
        if pre > 0.5:
            img.save('cache/Pick/1/%d_pick_%.3f.png' % (int(time()), pre))
        else:
            img.save('cache/Pick/0/%d_pick_%.3f.png' % (int(time()), pre))
    return round(pre), pre

def get_img(pos=None, sleep_time=0.5):
    '''return a raw PIL screenshot'''
    if pos is None: pos = cf_pos
    sleep(sleep_time)
    img = ImageGrab.grab(None, include_layered_windows=True, all_screens=True)
    img = Image.fromarray(np.array(img)[pos[1]:pos[3],pos[0]:pos[2]])
    img = img.resize((1920, 1080))
    sleep(sleep_time)
    return img

def get_sign(img,num):
    img = img.convert('L')
    img = img.resize((1920, 1080))
    img = np.array(img)
    temple = cv2.imread("./vision/templates/"+str(5)+".jpg", 0)
    th, tw = temple.shape[0],temple.shape[1]
    result = cv2.matchTemplate(img, temple, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    tl = max_loc
    # print(max_loc,"max_loc")
    br = (tl[0] + tw, tl[1] + th)
    if num == "1":
        x1 = int(((br[0] - tl[0]) * 0.2339) + tl[0])
        y1 = int(((br[1] - tl[1]) * 0.4953) + tl[1])
    if num == "2":
        x1 = int(((br[0] - tl[0]) * 0.4953) + tl[0])
        y1 = int(((br[1] - tl[1]) * 0.2339) + tl[1])
    if num == "3":
        x1 = int(((br[0] - tl[0]) * 0.7661) + tl[0])
        y1 = int(((br[1] - tl[1]) * 0.5047) + tl[1])
    if num == "4":
        x1 = int(((br[0] - tl[0]) * 0.5047) + tl[0])
        y1 = int(((br[1] - tl[1]) * 0.7661) + tl[1])
    #     xy = x1-1,y1-1
    # xy1 = x1+1,y1+1
    # # 绘制矩形框
    # cv2.rectangle(img, tl, br, (0, 0, 0), 2)
    # cv2.rectangle(img, xy, xy1, (0, 0, 0), 2)
    # # 设置显示窗口
    # cv2.namedWindow('match', 0)
    # # cv2.resizeWindow('match', 400, 600)
    # # 显示窗口
    # cv2.imshow('match', img)

    # # 结束
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    return x1, y1

def get_location(img):
    '''based on get_img, return relative location in proportion'''
    img = img.convert('L')
    img = img.resize((320, 180)) 
    img = transforms.ToTensor()(img).reshape((1, 1, 180, 320))
    img = -img + 1
    img = T.nn.functional.conv2d(img, mask, padding=19)
    img = img.reshape((180, 320)).numpy()
    img = (255*((img-img.min())/(img.max()-img.min()))**3).astype(np.uint8)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    _, seg = cv2.threshold(img, 150, 255, cv2.THRESH_BINARY)
    seg = cv2.erode(seg, kernel, iterations=2)
    seg = cv2.dilate(seg, kernel, iterations=3)
    # cv2.error might occur above
    col = np.nonzero(seg.sum(0))[0]
    col = col[col >= 320*col_ign]
    col = col[col < 320*(1-col_ign)]
    col = col[-1]
    row = np.nonzero(seg[:,col])[0]
    row = row[row >= 180*row_ign]
    row = row[row < 180*(1-row_ign)]
    row = row[len(row)//2]
    # IndexError may occur when seg is empty
    return np.array([row, col])/img.shape

def crop_PIL(img, loc, length, resize=None):
    '''crop square with length at loc and then resize (tuple)'''
    if resize is not None:
        img = img.resize(resize, Image.ANTIALIAS)
    row, col = (img.size[-1::-1]*loc).astype(int)
    img = img.crop((col-length/2, row-length/2, col+length/2, row+length/2))
    if cf.getint('basic', 'cache'):
        img.save('cache/%d_%s.png' % (int(time()),str(list(loc))))
    return img

def get_island_shot(pos=None, length=None, resize=(1920, 1080), loc=None):
    # 截岛部分的方形图
    img = get_img(pos)
    if length is None: length = get_crop_len()
    if isinstance(loc, list): 
        loc = np.array(loc)
        loc = loc/np.array([1080, 1920])
    if loc is None: loc = get_location(img)
    return crop_PIL(img, loc, length, resize), loc


def get_diff_img(img_before, img_after):
    # 灰度图差值的归一化，用于进行预测
    diff = np.array(img_before.convert('L'),'f')-np.array(img_after.convert('L'),'f')
    diff = 255*(((diff-diff.min())/(diff.max()-diff.min()+1e-4))**1)
    diff = diff.astype(np.uint8)
    diff = Image.fromarray(diff).convert('L')
    return diff


def predict(diff):
    '''
        判断自回复
        input：img_after-img_before
        return: 自回复置信度
    '''
    src = transforms.Resize((224, 224), interpolation=InterpolationMode.BILINEAR)(diff)
    src = transforms.ToTensor()(src).unsqueeze(0).to(device)
    pre = T.sigmoid(model_rcv(src)).cpu().item()
    print(pre)
    if cf.getint('basic', 'cache'):
        if pre >0.5:
            diff.save('cache/Recover/1/%d_recover_%.3f.png' % (int(time()), pre))
        else:
            diff.save('cache/Recover/0/%d_recover_%.3f.png' % (int(time()), pre))
    return pre

def markloc(a):
    x1, y1, z1, x2, y2, z2, x3, y3, z3 = a
    A = ((y3 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1))
    B = ((x3 - x1) * (z2 - z1)) - ((x2 - x1) * (z3 - z1))
    C = ((x2 - x1) * (y3 - y1)) - ((x3 - x1) * (y2 - y1))
    D = -(A * x1 + B * y1 + C * z1)
    return A, B, C, D

def new_z(q, loc):
    A, B, C, D = q
    x, y = loc
    i = A * x + B * y + D
    z = (i / C) * (-1)
    return z
