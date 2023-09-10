import torch as T
import imutils
import configparser
import copy
import time

global tip_location
tip_location = None
import torch
import os
import cv2
import numpy as np
import torch.nn as nn
import torch.nn.functional as F
# from torch.utils.data import Dataset, DataLoader
from torchvision import transforms, models
# from torchsummary import summary
# import random
# from predictor import get_img
from PIL import Image
import time
from .yolov5.detection import detect_island
from .yolov5.models.common import DetectMultiBackend
from .yolov5.utils.torch_utils import select_device

cf = configparser.ConfigParser()
cf.read('./vision/config.ini')
# cf.read("./config.ini")
trans = transforms.Compose([transforms.CenterCrop((360, 480)),
                            transforms.Grayscale(num_output_channels=1),
                            transforms.ToTensor()])
model_tip = models.shufflenet_v2_x1_5()
model_tip.conv1[0] = nn.Conv2d(1, 24, kernel_size=(3, 3), stride=(2, 2), padding=(1, 1), bias=False)
model_tip.fc = nn.Linear(1024, 2, bias=False)
model_tip.load_state_dict(T.load(cf.get('models', 'tip_loc'), map_location=T.device('cpu')))
model_tip.eval()

model_masked_loc = models.efficientnet_b0()
model_masked_loc.features[0][0] = nn.Conv2d(1, 32, kernel_size=(3, 3), stride=(2, 2), padding=(1, 1), bias=False)
model_masked_loc.classifier[1] = nn.Linear(1280, 2, bias=False)
model_masked_loc.load_state_dict(T.load(cf.get("models", "masked_loc"), map_location="cpu"))
model_masked_loc.eval()

device = select_device("cpu")
model_island = DetectMultiBackend(cf.get("models", "land"), device=device, dnn=False, data=None, fp16=False)


class Tip(object):
    def __init__(self, tip_loc, img, island_size, island_dist, material, process_cfg, mask_cfg, direction="down"):
        # scale: um:pixel;  resize: new shape of image(360, 480); crop tip vision from (360, 480)
        self.island_material = material
        self.scale = float(process_cfg["scale"])
        self.island_size = island_size / self.scale
        self.img = img.resize((1920, 1080), Image.NEAREST)

        img_copy = np.array(copy.copy(self.img))
        xywh = detect_island(model_island, img_copy, type="RGB")
        self.island_locs = [[x[1], x[0]] for x in xywh if np.abs(x[2]-x[3])/x[2] < 0.2]
        self.default = int((island_size+island_dist)/self.scale)
        self.min_move = 0.6*island_size/self.scale

        self.tip_loc = tip_loc

    def get_location(self, direction="down"):
        if direction == "up":
            candidates = [x for x in self.island_locs if
                          np.abs(x[0] - self.tip_loc[0] + self.default) < 0.6* self.default
                         and np.abs(x[1]-self.tip_loc[1]) < 0.6*self.default]
        elif direction == "down":
            candidates = [x for x in self.island_locs if
                          np.abs(x[0] - self.tip_loc[0] - self.default) < 0.6 * self.default
                         and np.abs(x[1]-self.tip_loc[1]) < 0.6*self.default]
        elif direction == "right":
            candidates = [x for x in self.island_locs if
                          np.abs(x[1] - self.tip_loc[1] - self.default) < 0.6 * self.default
                         and np.abs(x[0]-self.tip_loc[0]) < 0.6*self.default]
        else:
            raise NotImplementedError("direction must in [up, down, right]")
        if candidates:
            find_land = True
            is_loc = sort_with_dist(candidates, self.tip_loc)
            self.island_location = [is_loc[0]-3, is_loc[1]+2]
            relative_dist = [self.island_location[0] - self.tip_loc[0],
                             self.island_location[1] - self.tip_loc[1]]
        else:
            print("No island found in direction "+ direction)
            find_land = False
            if direction == "up":
                relative_dist = [-self.default, 0]
                self.island_location = [self.tip_loc[0]-self.default, self.tip_loc[1]]
            elif direction == "down":
                relative_dist = [self.default, 0]
                self.island_location = [self.tip_loc[0]+self.default, self.tip_loc[1]]
            elif direction == "right":
                relative_dist = [0, self.default]
                self.island_location = [self.tip_loc[0], self.tip_loc[1]+self.default]
        return find_land, relative_dist, self.tip_loc, self.island_location


def sort_with_dist(locs, target):
    assert locs
    dist = np.array([np.linalg.norm(np.array(x)- np.array(target)) for x in locs])
    index = np.argsort(dist)
    loc = locs[index[0]]
    return loc

def calibrate_tip_loc(image):
    img = np.array(image.resize((1920, 1080)))
    im_copy = copy.copy(img)
    # time.sleep(1)
    cv2.namedWindow("image")
    cv2.resizeWindow("image", 1920, 1080)
    os.makedirs("./cache/loc_img", exist_ok=True)

    def mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            xy = "%d,%d" % (x, y)
            cv2.circle(img, (x, y), 1, (255, 255, 255), thickness=-1)
            global tip_location
            tip_location = [y, x]
            # cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
            #             1.0, (255, 255, 255), thickness=1)

    cv2.setMouseCallback("image", mouse)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.imwrite("./cache/loc_img" + str(time.time()) + "_" + str(tip_location) + ".png",
                im_copy[:, :, ::-1].astype(np.uint8))
    return tip_location



def get_tip_loc():
    global tip_location
    return tip_location


def set_tip_loc(loc):
    global tip_location
    if isinstance(loc, np.ndarray):
        loc = loc.tolist()
    tip_location = loc


def get_relative_dist1(image):
    # get tip location
    # pixel:um = 72:6 island mask:79*79
    global tip_location
    image = image.convert("RGB").resize((1920, 1080), Image.NEAREST)
    data = trans(image).unsqueeze(0)
    tip_loc = model_tip(data)[0]
    tip_loc = np.array([int(tip_loc[0] * 360 + 360) - 4, int(tip_loc[1] * 480 + 480)])
    x, y = tip_loc[0], tip_loc[1]
    return x, y


def get_target_calibration(image_be, image_af):
    image = image_af.convert("RGB").resize((1920, 1080), Image.NEAREST)
    data = trans(image).unsqueeze(0)
    target_loc = model_masked_loc(data)[0]
    image = image_be.convert("RGB").resize((1920, 1080), Image.NEAREST)
    data = trans(image).unsqueeze(0)
    tip_loc = model_tip(data)[0]
    real_loc = (360 + int(360 * target_loc[0]), 480 + int(480 * target_loc[1]))
    print(tip_loc, target_loc)
    calibration = [int((tip_loc[0] - target_loc[0]) * 360) - 4,
                   int((tip_loc[1] - target_loc[1]) * 480)]
    os.makedirs("./cache/calibration", exist_ok=True)
    image_array = np.array(copy.copy(image))
    image_array[real_loc[0] - 2:real_loc[0] + 3, real_loc[1] - 2:real_loc[1] + 3, :] = [255, 255, 255]
    cv2.imwrite("./cache/calibration/" + time.strftime("%d-%H-%M-%S") + "_" + str(real_loc) + ".png",
                image_array[:, :, ::-1].astype(np.uint8))
    return calibration


def get_island_size(image):
    xywh = detect_island(model_island, np.array(image), type="RGB")
    island_size = [(x[2]+x[3])/2 for x in xywh if np.abs(x[2] - x[3]) / x[2] < 0.2]
    scale = float(cf.get('process', 'scale'))
    size = np.ceil(np.array(island_size).mean()*scale)
    assert 20 > size > 3, "island size is {}um, error".format(size)
    return size


def get_relative_dist(image, island_size, dist, material, direction="down"):
    # get tip location
    # pixel:um = 72:6 island mask:79*79
    global tip_location
    # os.makedirs("./cache/grep/", exist_ok=True)
    # cv2.imwrite("./cache/grep/" + time.strftime("%m%d_%H%M%S") + ".png", image[:, :, ::-1].astype(np.uint8))
    image = image.convert("RGB").resize((1920, 1080), Image.NEAREST) #PIL image
    tip_loc = get_tip_loc()
    # tip_img = image.rotate(90, expand=1)
    #
    # data = trans(tip_img).unsqueeze(0)
    # tip_loc = model_tip(data)[0]
    # print(tip_loc)
    # tip_loc = np.array([int(tip_loc[0] * 360 + 300) - 4, int(tip_loc[1] * 480 + 720)])
    # # tip_loc = np.array([int(tip_loc[0] * 360 + 360) - 4, int(tip_loc[1] * 480 + 720)])
    #
    # set_tip_loc(tip_loc)
    process = dict()
    mask = dict()
    for key, value in cf.items("process"):
        process[key] = value
    for key, value in cf.items("calibrator_mask"):
        mask[key] = value
    nm, um, mm = 1, 1000, 1000000
    tip = Tip(tip_loc, image, island_size, dist, material, process_cfg=process, mask_cfg=mask, direction=direction)
    find_land, relative_dist, _, island_location = tip.get_location(direction)

    [land_row, land_col] = island_location
    [row, col] = tip_loc
    o_image = np.array(image.resize((1920, 1080), Image.NEAREST))
    o_image[row - 4: row + 4, col - 4:col + 4, :] = np.array([255, 255, 0])
    if land_row is not None:
        o_image[land_row - 4:land_row + 4, land_col - 4:land_col + 4, :] = np.array([0, 0, 255])
    os.makedirs("./cache/o_img", exist_ok=True)
    # print(time.strftime("%m%d_%H%M%S"))
    cv2.imwrite("./cache/o_img/" + time.strftime("%H%M%S") + "_{}_{}.png"
                .format(direction, find_land), o_image[:, :, ::-1].astype(np.uint8))
    land_row = int(relative_dist[0] * tip.scale * um)
    land_col = int((relative_dist[1] * tip.scale) * um)
    return land_row, land_col, find_land


def create_direction_matrix(row, col):
    direction = np.ones([row, col])
    direction[:, 1:col:2] = -1 * direction[:, 1:col:2]
    direction[-1, :] = 0
    return direction


if __name__ == "__main__":
    path = "./cache/09-16-52-37_(541, 635).png"
    Img = Image.open(path)
    get_target_calibration(Img, Img)
    # for file in os.listdir(path)[10:20]:
    #     print(file)
    #     img = Image.open(os.path.join(path, file))
    #     for direction in ["up", "right", "down"]:
    #         # time.sleep(1)
    #         get_relative_dist(img, 10, 20, material="65", direction=direction)
    # image = cv2.imread("./o_img1654590828.7276309.png")
    # img_c = image.copy()
    # # device = select_device("cpu")
    # # model = DetectMultiBackend("./yolov5/best.pt", device=device, dnn=False, data=None, fp16=False)
    # results = detect_island(model_island, image)
    # loc = [[x[1], x[0]] for x in results if np.abs(x[2]-x[3])/x[2]<0.2]
    # for x in loc:
    #     img_c[x[0] - 1:x[0] + 2, x[1] - 1:x[1] + 2, :] = [255, 0, 255]
    # cv2.imwrite("final.png", img_c)
    # print(loc)


