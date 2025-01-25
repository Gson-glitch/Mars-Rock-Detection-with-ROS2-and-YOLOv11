# Mars Rock Detection with ROS2 and YOLOv11

This project implements a rock detection system using ROS2 and YOLOv11. The system detects rocks in video input and controls a turtlesim robot to move toward the largest detected rock.

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble - [Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Python 3.8 or later
- Conda (for environment management)

## Project Structure

```
ROS_YOLO/
├── trained_yolov11/             # Trained YOLO model
│   └── best.pt
└── ros2_ws/                     # ROS2 workspace
    └── src/
        └── rock_detection/      # ROS2 package
            ├── launch/
            │   └── rock_detection.launch.py
            ├── rock_detection/
            │   ├── models/
            │   │   └── best.pt
            │   ├── rock_detector.py
            │   └── video_publisher.py
            ├── setup.py
            └── package.xml
```

## Setup

1. Create and activate conda environment:

```bash
cd ROS_YOLO
conda env create -f env.yml
conda activate ros2-yolov11
```

2. Build the ROS2 package:

```bash
cd ros2_ws
colcon build --packages-select rock_detection
```

## Running the System

### Terminal 1: Launch the main system

```bash
cd ROS_YOLO
source ros2_ws/install/setup.bash
# Run with webcam:
ros2 launch rock_detection rock_detection.launch.py
# OR run with video file:
ros2 launch rock_detection rock_detection.launch.py video_source:="path/to/your_video.mp4"
```

### Terminal 2: View detection results

```bash
cd ROS_YOLO
source ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Select the `/rock_detections` topic from the dropdown menu.

### Terminal 3 (Optional): Modify parameters

```bash
cd ROS_YOLO
source ros2_ws/install/setup.bash
# Adjust turtle movement parameters
ros2 param set /rock_detector min_distance_to_wall 1.5
ros2 param set /rock_detector turtle_linear_speed 0.3
ros2 param set /rock_detector turtle_angular_scale 0.8
```

## System Components

- **Video Publisher Node**: Publishes video frames from webcam or file
- **Rock Detector Node**: Processes frames using YOLOv11 for rock detection
- **Turtlesim Node**: Simulated robot that moves toward detected rocks

## Parameters

### Rock Detector Parameters

- `min_distance_to_wall`: Minimum distance to keep from walls (default: 1.5)
- `turtle_linear_speed`: Forward movement speed (default: 0.3)
- `turtle_angular_scale`: Turning speed scale (default: 0.8)
- `confidence_threshold`: Detection confidence threshold (default: 0.5)

## Topics

- `/video_stream`: Raw video input
- `/rock_detections`: Processed video with detection visualization
- `/turtle1/cmd_vel`: Turtle movement commands

## Dependencies

- rclpy
- sensor_msgs
- geometry_msgs
- cv_bridge
- OpenCV
- Ultralytics (YOLOv11)
- Python3-opencv
