#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import cv2
import torch
import statistics
import numpy as np

from numpy import random
from cv_bridge import CvBridge
from models.experimental import attempt_load
from utils.datasets import letterbox
from utils.general import check_img_size, check_requirements, non_max_suppression, scale_coords
from utils.plots import plot_one_box
from utils.torch_utils import select_device, time_synchronized

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool

WEIGHTS = 'weights/tf_best.pt'
IMG_SIZE = 640
DEVICE = ''
AUGMENT = False
CONF_THRES = 0.50
IOU_THRES = 0.45
CLASSES = None
AGNOSTIC_NMS = False

QUEUE_SIZE = 13
CLASS_MAP = ['Green', 'Left', 'Red', 'Straightleft', 'Yellow']

# Initialize
device = select_device(DEVICE)
half = device.type != 'cpu'  # half precision only supported on CUDA
print('device:', device)

# Load model
model = attempt_load(WEIGHTS, map_location=device)  # load FP32 model
stride = int(model.stride.max())  # model stride
imgsz = check_img_size(IMG_SIZE, s=stride)  # check img_size
if half:
    model.half()  # to FP16

# Get names and colors
names = model.module.names if hasattr(model, 'module') else model.names
colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

# Run inference
if device.type != 'cpu':
    model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once

class YOLOv7():
    def __init__(self):
        self.set_attribute()
        self.set_ros()
    
    def set_attribute(self):
        self.yolo_mode = True

        self.queue_list = [[0 for i in range(QUEUE_SIZE)] for j in range(5)]
        return
    
    def set_ros(self):
        # ROS
        rospy.init_node('traffic_light_detection')

        self.img_sub = rospy.Subscriber('/image/traffic_light', Image, self.callback_img)
        self.traffic_mode_sub = rospy.Subscriber('/mode/traffic', Bool, self.callback_traffic_mode)

        self.img_res_pub = rospy.Publisher('/yolo/traffic_light', Image, queue_size=10)
        self.traffic_pub = rospy.Publisher('/traffic_sign', String, queue_size=10)

        self.yolo_timer = rospy.Timer(rospy.Duration(0.1), self.callback_traffic_pub)
        return
    
    def callback_img(self, img_raw_msg):
        with torch.no_grad():
            bridge = CvBridge()
            cap = bridge.imgmsg_to_cv2(img_raw_msg, desired_encoding='bgr8')

            if self.yolo_mode is True:
                img_res, ids = self.detect(cap)
                img_res_msg = bridge.cv2_to_imgmsg(img_res, encoding="bgr8")

            else:
                img_res_msg = bridge.cv2_to_imgmsg(cap, encoding="bgr8")

            self.img_res_pub.publish(img_res_msg)
        return
    
    def detect(self, img0):
        # Padded resize
        img = letterbox(img0, imgsz, stride=stride)[0]

        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t0 = time_synchronized()
        pred = model(img, augment=AUGMENT)[0]

        # Apply NMS
        pred = non_max_suppression(pred, CONF_THRES, IOU_THRES, classes=CLASSES, agnostic=AGNOSTIC_NMS)

        # Process detections
        det = pred[0]
        numClasses = len(det)

        s = ''
        s += '%gx%g ' % img.shape[2:]  # print string

        ids = []
        if numClasses:
            # Rescale boxes from img_size to img0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()

            # Print results
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
                s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

            # Write results
            for *xyxy, conf, cls in reversed(det):
                id = int(cls)
                ids.append(id)
                label = f'{names[id]} {conf:.2f}'
                plot_one_box(xyxy, img0, label=label, color=colors[id], line_thickness=3)
                xmin, ymin, xmax, ymax = [int(tensor.item()) for tensor in xyxy]

                ymean = (ymin + ymax) // 2
                self.queue_list[id].append(1)
                # if ymean < self.img_height / 2:
                #     self.queue_list[id].append(1)
                # else:
                #     self.queue_list[id].append(0)

            ids = list(set(ids))
            for i in range(5):
                if i not in ids:
                    self.queue_list[i].append(0)

        else:
            for k in range(5):
                self.queue_list[k].append(0)

        return img0, ids
    
    def hard_vote(self, queue):
        if sum(queue) > 0.7 * QUEUE_SIZE:
            return True

        else:
            return False

    def callback_traffic_pub(self, event):
        final_check = String()
        data = ""

        for n in range(5):
            while len(self.queue_list[n]) != QUEUE_SIZE: # delete first element
                del self.queue_list[n][0]

            queue_list = self.queue_list[n]

            # queue voting
            if self.hard_vote(queue_list):
                if data == "":
                    data += CLASS_MAP[n]
                else:
                    data += "," + CLASS_MAP[n]

        if data == "":
            data = "None"

        print('Traffic Light : ' + data)

        final_check.data = data

        self.traffic_pub.publish(final_check)
        return
    
    def callback_traffic_mode(self, tm_msg):
        self.yolo_mode = tm_msg.data
        
if __name__ == "__main__":
    try:
        # ROS
        yolo = YOLOv7()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
