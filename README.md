Probabilistic_semantic_mapping

Stacks include:
  - `aizo_quadrotor_slam`: 2D SLAM based on CaroGrapher and TEB motion planning
  - `oj_detection`: tracker and extracted object, real-time object detection using the Ultralytics YOLO
  - The `object_detector_node` provides real-time object detection on incoming ROS image messages using the Ultralytics YOLO model.
  - The `tracker_with_cloud_node` provides functionality for 3D object detection by integrating 2D detections, LiDAR data, and camera information.
Check each package for more details.

# Our proposed:
<img src="https://github.com/NguyenCanhThanh/probabilistic_semantic_mapping/blob/main/image/proposed_system.png" width="450px">

|  `input image`  |  `point cloud`  | `tracker object`  |  `cluster point`  | `traditional map`  |  `semantic map`  |
| :------------: | :-----------------------: | :------------: | :-----------------------: | :------------: | :-----------------------: |
| <img src="https://github.com/NguyenCanhThanh/probabilistic_semantic_mapping/blob/main/image/input1.png" width="150px"> | <img src="https://github.com/NguyenCanhThanh/probabilistic_semantic_mapping/blob/main/image/point1.png" width="150px"> | <img src="https://github.com/NguyenCanhThanh/probabilistic_semantic_mapping/blob/main/image/detect1.png" width="150px"> | <img src="https://github.com/NguyenCanhThanh/probabilistic_semantic_mapping/blob/main/image/cluster1.png" width="150px"> | <img src="https://github.com/NguyenCanhThanh/probabilistic_semantic_mapping/blob/main/image/metricavoid1.png" width="150px"> | <img src="https://github.com/NguyenCanhThanh/probabilistic_semantic_mapping/blob/main/image/ouravoid2.png" width="150px">|

Citation: 
```
Object-Oriented Semantic Mapping for Reliable UAVs Navigation, ICCAIS 2023
```

## Setup ⚙
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/NguyenCanhThanh/probabilistic_semantic_mapping.git
$ python3 -m pip install -r ultralytics_ros/requirements.txt
$ cd ~/catkin_ws
$ rosdep install -r -y -i --from-paths .
$ catkin build
```

## Requirements
Requirements
 - `ros` (indigo+)
 - `gazebo` (2.2+)
 - `gazebo_ros` (2.2+)
 - `quadrotor_control` [KumarRobotics](https://github.com/KumarRobotics/quadrotor_control)
 - `qudrotor_msgs` [KumarRobotics](https://github.com/KumarRobotics/kr_planning_msgs)
 - `waypoint_navigation` [](https://github.com/KumarRobotics/waypoint_navigation_plugin).
 - `aizo_quadrotor` [NCT](https://github.com/NguyenCanhThanh/aizo_quadrotor.git)

## Step 1: Custom dataset
## Step 2: SLAm node

```
- CartoGrapher build static map
roslaunch aizo_quadrotor_slam 2dslam.launch

- Move based navigation using TEB

roslaunch aizo_quadrotor_slam move_base.launch 
```

## `tracker_node`
### Params
- `yolo_model`: Pre-trained Weights.  
For yolov8, you can choose `yolov8*.pt`, `yolov8*-seg.pt`, `yolov8*-pose.pt`.

  |  YOLOv8  |  <img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/08770080-bf20-470b-8269-eee7a7c41acc" width="350px">  |
  | :-------------: | :-------------: |
  |  **YOLOv8-seg**  |  <img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/7bb6650c-769d-41c1-86f7-39fcbf01bc7c" width="350px">  |
  |  **YOLOv8-pose**  |  <img src="https://github.com/Alpaca-zip/ultralytics_ros/assets/84959376/46d2a5ef-193b-4f83-a0b3-6cc0d5a3756c" width="350px">  |
  
  See also: https://docs.ultralytics.com/models/
- `image_topic`: Topic name for image.
- `detection_topic`: Topic name for 2D bounding box.
- `conf_thres`: Confidence threshold below which boxes will be filtered out.
- `iou_thres`: IoU threshold below which boxes will be filtered out during NMS.
- `max_det`: Maximum number of boxes to keep after NMS.
- `tracker`: Tracking algorithms.
- `classes`: List of class indices to consider.  
See also: https://github.com/ultralytics/ultralytics/blob/main/ultralytics/datasets/coco128.yaml 
- `debug`:  If true, run simple viewer.
- `debug_conf`:  Whether to plot the detection confidence score.
- `debug_line_width`: Line width of the bounding boxes.
- `debug_font_size`: Font size of the text.
- `debug_labels`: Font to use for the text.
- `debug_font`: Whether to plot the label of bounding boxes.
- `debug_boxes`: Whether to plot the bounding boxes.
### Topics
- Subscribed Topics:
  - Image data from `image_topic` parameter. ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- Published Topics:
  - Debug images to `/debug_image` topic. ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
  - Detected objects(2D bounding box) to `detection_topic` parameter. ([vision_msgs/Detection2DArray](http://docs.ros.org/en/melodic/api/vision_msgs/html/msg/Detection2DArray.html))
## `tracker_with_cloud_node`
### Params
- `camera_info_topic`: Topic name for camera info.
- `point_topic`: Topic name for point cloud.
- `detection2d_topic`: Topic name for 2D bounding box.
- `detection3d_topic`: Topic name for 3D bounding box.
- `cluster_tolerance`: Spatial cluster tolerance as a measure in the L2 Euclidean space.
- `min_cluster_size`: Minimum number of points that a cluster needs to contain.
- `max_cluster_size`: Maximum number of points that a cluster needs to contain.
### Topics
- Subscribed Topics:
  - Camera info from `camera_info_topic` parameter. ([sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html))
  - Lidar data from `point_topic` parameter. ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))
  - Detected objects(2D bounding box) from `detection2d_topic` parameter. ([vision_msgs/Detection2DArray](http://docs.ros.org/en/melodic/api/vision_msgs/html/msg/Detection2DArray.html))
- Published Topics:
  - Detected cloud points to `/detection_cloud` topic. ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))
  - Projected cloud points to `/projection_cloud` topic. ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))
  - Detected objects(3D bounding box) to `detection3d_topic` parameter. ([vision_msgs/Detection3DArray](http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3DArray.html))
  - Visualization markers to `/detection_marker` topic. ([visualization_msgs/MarkerArray](https://docs.ros.org/en/api/visualization_msgs/html/msg/MarkerArray.html))
