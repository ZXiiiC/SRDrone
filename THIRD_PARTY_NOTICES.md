# Third-party source and assets

The root Apache-2.0 license covers SRDrone-authored code, including this reconstruction. It does not replace third-party notices or grant rights to third-party assets.

- `object_det/scripts/models/{common,yolo}.py` and `object_det/scripts/utils/{general,torch_utils}.py` retain YOLOv5 / Ultralytics GPL-3.0 headers. Their license text is included at `LICENSES/YOLOv5-GPL-3.0.txt`, obtained from the [YOLOv5 v6.2 source](https://github.com/ultralytics/yolov5/tree/v6.2). The precise upstream revision of the pre-existing copied files has not been recovered. These files are not included in the standalone Python runtime wheel.
- `object_det/scripts/Model/*.pt` are pre-existing repository assets. Their training data, training recipes, exact source and separate license grants were not supplied in the recovery materials. This reconstruction does not add or retrain model binaries.
- BehaviorTree.CPP, ROS, MAVROS, OpenCV, PCL, PX4, RflySim, Sentence Transformers and the MPNet model are separate dependencies. Consult their distributions for their own licenses and setup requirements.
- The IEEE publication PDF and the full personal research notebook are not redistributed. Only the author's selected prompt excerpts and one notebook case are included, with provenance. Private document image links and unrelated notebook material are omitted.
