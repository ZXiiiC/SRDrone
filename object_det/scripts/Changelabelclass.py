#!C:\pythonCode
# -*- coding: utf-8 -*-
# @Time : 2023/9/1 15:28
# @Project : Yolo5_Detect
# @File : Changelabelclass.py
# @Author : YDYH
# @Software: PyCharm
import os

# 待更改标签的文件名称
folder_path = 'D:/Programming/datasets/Balloon/labels'
label_file = os.listdir(folder_path)

class_path = 'D:/Programming/datasets/Balloon/classes.txt'
with open(class_path, 'r', encoding='utf-8') as file:
    classes = file.readlines()
nc = len(classes)

for label in label_file:
    label_path = os.path.join(folder_path, str(label))
    with open(label_path, 'r', encoding='utf-8') as file:
        labels = file.readlines()

    anno_mess = []
    for line in labels:
        # line = line.replace('"', '').replace(',', '').replace('[', '').replace(']', '')
        line = [num for num in line.strip().split()]
        if nc == 1 and line[0] != 0:
            line[0] = '0'
        anno_mess.append([int(line[0]), float(line[1].strip("'")), float(line[2].strip("'")),
                          float(line[3].strip("'")), float(line[4].strip("'"))])
    with open(label_path, 'w') as file:  # 保存标签信息到对应txt文件中
        for sublist in anno_mess:
            line = ' '.join(str(item) for item in sublist)
            file.write(line + '\n')


