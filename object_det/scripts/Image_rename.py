#!C:\pythonCode
# -*- coding: utf-8 -*-
# @Time : 2023/9/1 11:07
# @Project : Yolo5_Detect
# @File : Image_rename.py
# @Author : YDYH
# @Software: PyCharm

import os

# 更改图片的文件名称
# 指定文件夹路径和新文件名前缀
folder_path = 'D:/Programming/datasets/Balloon/Image_collect'
new_name_prefix = 'Balloon'

# 获取文件夹中所有图片文件的路径
file_names = os.listdir(folder_path)
file_names.sort(key=lambda x: int(''.join(filter(str.isdigit, x))))
image_files = [os.path.join(folder_path, f) for f in file_names if f.endswith(('.jpg', '.jpeg', '.png', '.gif'))]

cnt = 196
# 遍历所有图片文件，重命名并移动到同一文件夹下
for i, image_file in enumerate(image_files):
    # 构建新文件名
    new_name = new_name_prefix + str(cnt) + os.path.splitext(str(image_file))[1]
    cnt += 1
    # 重命名文件
    os.rename(image_file, os.path.join(folder_path, new_name))