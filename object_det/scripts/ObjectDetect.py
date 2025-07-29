#!C:\pythonCode
# -*- coding: utf-8 -*-
# @Time : 2023/4/1 19:49
# @Author : YDYH
# @File : ObjectDetect.py
# @Software: PyCharm
'''
Usage - sources:
    $ python Object_detect/ObjectDetect.py --weights Object_detect/circle.pt --source 0                  # webcam
                                                                                      img.jpg            # image
                                                                                      vid.mp4            # video
'''

IMG_FORMATS = 'bmp', 'dng', 'jpeg', 'jpg', 'mpo', 'png', 'tif', 'tiff', 'webp', 'pfm'  # include image suffixes
VID_FORMATS = 'asf', 'avi', 'gif', 'm4v', 'mkv', 'mov', 'mp4', 'mpeg', 'mpg', 'ts', 'wmv'  # include video suffixes

import torch
import torch.nn as nn

import random
import cv2
import numpy as np
import glob
import os
from pathlib import Path
import sys

# 十分必要，否则在其他项目中应用时可能报错
FILE = Path(__file__).resolve()  # 解析当前脚本文件所在的绝对路径并保存到FILE变量中
ROOT = FILE.parents[0]  # 获取当前脚本文件所在的父目录
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # 将当前脚本文件所在父目录添加到系统搜索路径中

from utils.general import non_max_suppression, check_img_size, scale_boxes, LOGGER, increment_path, Profile


class Yolo_Detect:
    def __init__(self, model_path='yolov5s.pt', device='cuda', img_size=640, conf_thres=0.25, iou_thres=0.45):
        '''
        Args:
            model_path: 模型所在路径
            device: 检测使用的设备
            img_size: 图片大小
            conf_thres: 置信度阈值
            iou_thres: iou阈值
        '''
        self.model_path = model_path
        self.img_size = check_img_size(img_size, s=32)
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres

        # 加载模型
        # 判断检测能否使用cuda进行检测，如果不能就用‘cpu’
        if torch.cuda.is_available():
            self.device = device
            self.device = torch.device(self.device)
        else:
            self.device = 'cpu'
            self.device = torch.device(self.device)

        LOGGER.info(f'device:{self.device}')
        # 加载模型
        self.model = torch.load(self.model_path, map_location=self.device)
        self.model = self.model['model'].float().fuse().eval()
        # model inplace设置
        for m in self.model.modules():
            if type(m) in [nn.Hardswish, nn.LeakyReLU, nn.ReLU, nn.ReLU6]:
                m.inplace = True  # torch version >= 1.70
        # 获取labels
        self.labels = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        # 随机得到colors
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(self.labels))]

    def plot_one_box(self, x, img, color=None, label=None, line_thickness=None):
        '''
        Args:
            x: 目标框
            img: 需要绘制边界框的图片
            color: 图片
            label: 标签名称
            line_thickness: 线宽
        '''
        # Plots one bounding box on image img
        tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
        color = color or [random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
        if label:
            tf = max(tl - 1, 1)  # font thickness
            t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
            cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
            cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

    def img_preprocessing(self, img):
        '''
        Args:
            img: 原图
        Returns:
            img: 预处理后的图片
        '''

        # resize img
        img = letterbox(img, new_shape=self.img_size)[0]
        # BGR(H W C) -> RGB(C H W)
        # 转换图片的样式
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)  # 将图像转换为C语言那样的连续存储格式，以提高图片处理速率
        img = torch.from_numpy(img).to(self.device)  # 将图像数据类型从ndarray转换为tensor格式
        img = img.float()
        # 归一化
        img /= 255.0
        # (C H W) -> (1 C H W)
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        return img

    def __call__(self, img):
        '''
        Args:
            img: 待检测的图片
        Returns:
            img0: 检测完成并标记成功的图片，ndarry BGR(H W C) ,
            det: [(x1, y1, x2, y2, conf, cls),....]
            dt: tuple 记录了处理一张图片三个阶段（预处理、前向推理、后处理）分别用去的时间
        '''

        img0 = img.copy()

        dt = (Profile(), Profile(), Profile())  # 记录一次预测各个处理阶段所用的时间

        # img 预处理
        with dt[0]:
            img = self.img_preprocessing(img)

        # 模型前向推理
        with dt[1]:
            out = self.model(img)[0]

        # 非极大值抑制
        with dt[2]:
            det = non_max_suppression(out, self.conf_thres, self.iou_thres)[0]

        # Rescale boxes from img_size to img0 size
        if det is not None and len(det):
            det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], img0.shape).round()
            # 绘制box
            for *xyxy, conf, cls in reversed(det):
                label = '%s %.2f' % (self.labels[int(cls)], conf)
                # 绘制boxs
                self.plot_one_box(xyxy, img0, label=label, color=self.colors[int(cls)], line_thickness=3)

        return img0, det.cpu().numpy(), dt


def letterbox(im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better val mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return im, ratio, (dw, dh)


def save_result(save_dir, path, img, det):
    for i in range(len(img)):
        p = Path(path[i])
        save_path = str(save_dir / p.name)  # im.jpg
        txt_path = str(save_dir / 'labels' / p.stem)
        LOGGER.info(f"detect results have been saved in {save_path, txt_path}")
        # 首先将det（检测边界框的结果数据类型转换为ndarray的数据类型）
        det_n = np.round(np.array(det[i]).astype(float), 3)  # 转换前，需要先转换为‘cpu’存储数据
        det_str = '\n'.join('\t'.join(map(str, row)) for row in det_n)

        txt_path = txt_path + '.txt'

        cv2.imwrite(save_path, img[i])
        with open(txt_path, 'w') as f:
            f.write(det_str)
        f.close()


def Load_images(path):
    if isinstance(path, str) and Path(path).suffix == '.txt':  # *.txt file with img/vid/dir on each line
        path = Path(path).read_text().rsplit()
    files = []
    for p in sorted(path) if isinstance(path, (list, tuple)) else [path]:
        p = str(Path(p).resolve())
        if '*' in p:
            files.extend(sorted(glob.glob(p, recursive=True)))  # glob
        elif os.path.isdir(p):
            files.extend(sorted(glob.glob(os.path.join(p, '*.*'))))  # dir
        elif os.path.isfile(p):
            files.append(p)  # files
        else:
            raise FileNotFoundError(f'{p} does not exist')

    images = [x for x in files if x.split('.')[-1].lower() in IMG_FORMATS]
    return images


if __name__ == '__main__':
    # opt = parse_opt()
    # Detect(**vars(opt))

    conf_thres = 0.25  # 置信度阈值
    iou_thres = 0.45  # iou阈值
    # model_path = 'Model/yolov5s.pt'
    # model_path = 'Model/UAVDT-TRT8.6.1/UAVDT-NEW.pt'
    model_path = 'Model/UAVDT-TRT8.6.1/UAVDT-Pruned.pt'
    device = 'cuda'  # or cpu
    img_size = 640

    source = r'data/videos/test.mp4'
    # source = r'data/images'
    # source = 0

    # source = r'data/uav_circle'
    source = str(source)
    save_r = True

    is_img = Path(source).suffix[1:] in IMG_FORMATS  # 是否为图片
    is_vid = Path(source).suffix[1:] in VID_FORMATS  # 是否为视频
    is_cam = source.isnumeric()  # 是否进行实时检测

    save_dir = increment_path(Path('runs/detect') / 'exp', exist_ok=False)  # increment run
    (save_dir / 'labels').mkdir(parents=True, exist_ok=True)  # make dir

    detect = Yolo_Detect(model_path, device, img_size, conf_thres, iou_thres)

    if is_cam:
        # 实时检测
        cap = cv2.VideoCapture(0)
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(str(save_dir / 'result.mp4'), fourcc, 30, (width, height))
        time_t = []  # 记录每处理一张图片所花的时间
        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                frame0, det, dt = detect(frame)
                t = tuple(x.t * 1E3 for x in dt)
                time_t.append(t)
                LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS.' % t)
                det_n = det
                cv2.imshow('img', frame0)
                out.write(frame0)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                break
        mean_time = tuple(np.mean(np.array(time_t), axis=0))
        LOGGER.info(f'\nSpeed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image.' % mean_time)
        cap.release()
        out.release()
        cv2.destroyAllWindows()
    elif is_vid:
        cap = cv2.VideoCapture(source)
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')

        out = cv2.VideoWriter(str(save_dir / 'result.mp4'), fourcc, 30, (width, height))
        time_t = []  # 记录每处理一张图片所花的时间
        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                frame0, det, dt = detect(frame)
                t = tuple(x.t * 1E3 for x in dt)
                time_t.append(t)
                LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS.' % t)
                # cv2.imshow('img', frame0)
                out.write(frame0)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                break
        mean_time = tuple(np.mean(np.array(time_t), axis=0))
        LOGGER.info(f'\nSpeed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image.' % mean_time)
        LOGGER.info(f"detect results have been saved in {str(save_dir / 'result.mp4')}")
        cap.release()
        out.release()
    else:
        images = Load_images(source)
        img_l, det_l = [], []
        time_t = []  # 记录每处理一张图片所花的时间
        for i in images:
            img = cv2.imread(i, 1)
            img0, det, dt = detect(img)
            det = det.cpu()
            img_l.append(img0)
            det_l.append(det)
            t = tuple(x.t * 1E3 for x in dt)
            time_t.append(t)
            LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS.' % t)
            # 显示img
            if len(images) == 1:
                cv2.imshow('img', img0)
                if cv2.waitKey() == ord('q'):  # press q to next img
                    pass
        mean_time = tuple(np.mean(np.array(time_t), axis=0))
        LOGGER.info(f'\nSpeed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image.' % mean_time)
        save_result(save_dir, images, img_l, det_l)
