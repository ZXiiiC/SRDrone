# 使用方法

## `ObjectDetect.py`

**使用方法：**拷贝整个文件目录当项目文件夹中，在需要使用的`python`文件中加入下面的一行：

```python
# 导入检测类
from Yolov5_Detect.ObjectDetect import Yolo_Detect

# 使用方法：
# 1.定义检测对象：model_path：模型文件位置，str  要求：模型文件是pytorch架构，以.pt为后缀名
#			   device：计算用设备，可选参数，str(例如：'cuda:0'、'cpu')  默认值：'cuda'  
#   		   img_size：图片大小，可选参数，int  默认值：640
#              conf_thres：置信度阈值，可选参数，float  默认值：0.25
#              iou_thres：IOU阈值，可选参数，float   默认值：0.45
detect = Yolo_Detect(model_path, device, img_size, conf_thres, iou_thres)

# 2.使用检测对象：
#        Args:
#            img0: 待检测的图片
#        Returns:
#            img: 检测完成并标记成功的图片，ndarry BGR(H W C) ,
#            det: [(x1, y1, x2, y2, conf, cls),....]
#            dt: tuple 记录了处理一张图片三个阶段（预处理、前向推理、后处理）分别用去的时间
img, det, dt = detect(img0)
```





## `ObjectDetect_TensorRT.py`

**使用方法：**拷贝整个文件目录当项目文件夹中，在需要使用的`python`文件中加入下面的一行：

```python
# 导入检测类
from Yolov5_Detect.ObjectDetect_TensorRT import Yolo_Detect

# 使用方法：
# 1.定义检测对象：model_path：模型文件位置，str  要求：模型文件是TensorRT架构，以.engine为后缀名
#			   device：计算用设备，可选参数，str(例如：'cuda:0'、'cpu')  默认值：'cuda'  
#   		   img_size：图片大小，可选参数，int  默认值：640
#              conf_thres：置信度阈值，可选参数，float  默认值：0.25
#              iou_thres：IOU阈值，可选参数，float   默认值：0.45
detect = Yolo_Detect(model_path, device, img_size, conf_thres, iou_thres)

# 2.使用检测对象：
#        Args:
#            img0: 待检测的图片
#        Returns:
#            img: 检测完成并标记成功的图片，ndarry BGR(H W C) ,
#            det: [(x1, y1, x2, y2, conf, cls),....]
#            dt: tuple 记录了处理一张图片三个阶段（预处理、前向推理、后处理）分别用去的时间
img, det, dt = detect(img0)
```



**注：**这两个代码在检测时牺牲了一点预处理图片的性能（比`YOLOv5`源代码预处理部分慢），暂时还没来得及修改。
