This repository contains ros2 packages.

# **Object Tracking in ROS2**

This project implements **real-time object detection and tracking** using **ROS2 (Humble)**. The system captures images from a **live camera or video**, processes them through a **YOLO V8 model**, and tracks objects using a **Centroid Tracking Algorithm**.

---

## **Nodes & Topics**
This project consists of the following ROS2 nodes:

### **Image Publisher (`image_publisher`)**
- **Publishes:** `/camera/image_raw`  
- **Streams images** from a live camera or video file.

### **Image Subscriber (`image_subscriber`)**
- **Subscribes:** `/camera/image_raw`  
- **Publishes:** `/object_detection`  
- **Runs YOLOv8** to detect objects and outputs bounding boxes.

### **Object Tracking (`object_tracking`)**
- **Subscribes:** `/object_detection`  
- **Publishes:** `/object_tracking`  
- **Tracks objects** across frames using Centroid Tracking.

### **Visualization (`visualization`)**
- **Subscribes:** `/camera/image_raw`, `/object_detection`, `/object_tracking`  
- **Publishes:** `/visualization/image`  
- **Overlays detections and tracking results** on images.

---

## **Project Structure**

``` bash

ros2_ws/src/
│── object_tracking_viz/
│   ├── config/                         # YAML configuration files
│   │   ├── params.yaml                 # ROS2 parameters (camera, video file, tracking settings)
│   ├── include/object_tracking_viz/    # Cpp header files
│   │   ├── header.cpp         
│   ├── launch/                         # ROS2 launch files
│   │   ├── app.launch.py               # Launch all nodes
│   ├── media/                          # Sample video files for testing
│   │   ├── test1.mp4
│   │   ├── test2.mp4
│   │   ├── test3.mp4
│   ├── object_tracking_viz/            # Python packages
│   │   ├── __init__.py
│   ├── scripts/                        # Python scripts
│   │   ├── image_subscriber.py         # Subscribes images and detect objects using YOLOv8
│   │   ├── object_visualization.py     # Overlays tracking & detection results
│   ├── src/                            # C++ source files
│   │   ├── image_publisher.cpp         # Publishes images from camera or video
│   │   ├── object_tracking.cpp         # Tracks objects using Centroid Tracking
│   ├── CMakeLists.txt                  # ROS2 build configuration
│   ├── package.xml                     # Package dependencies
│── .gitignore
│── README.md                           # Project documentation

```
---

## **Installation**
Clone this repository into your ROS2 workspace.

Install the ROS2 Hunble distribution:

``` bash
https://docs.ros.org/en/humble/Installation.html
```

## **Dependencies**
This project requires **ROS2 Humble**, along with additional system and Python dependencies.

### **System Dependencies**
Install the required system packages:

```bash
sudo apt update && sudo apt install -y \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-ament-index-cpp \
    libeigen3-dev
```

### **Python Dependencies**
Install the required Python packages:

```bash
pip install opencv-python numpy ultralytics nlohmann-json
```

## **Build the Package**

``` bash
cd ~/ros2_ws
colcon build --packages-select object_tracking_viz
source install/setup.bash
```

## **Configuration (params.yaml)**
Modify config/params.yaml to adjust the camera usage, video file name, and tracking settings

## **Running the Nodes**
Use the launch file to start all ROS2 nodes:

``` bash
ros2 launch object_tracking_viz app.launch.py
```

OR, manually run each node:

``` bash
ros2 run object_tracking_viz image_publisher
ros2 run object_tracking_viz image_subscriber
ros2 run object_tracking_viz object_tracking
ros2 run object_tracking_viz object_visualization
```

---

## **Resources & References**
Here are some helpful resources related to using **WSL2**, **USB support in WSL2**, **Gazebo Simulation**, and **OpenCV VideoIO**:

### ROS2:
- [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)
- [ROS2 Tutorials](https://youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy&si=sx0AHXzXGNbZhxi1)
- [Camera usage in ROS2](https://youtu.be/6e94ZnYnO_U)

### WSL2 & USB Camera in WSL2:
- [Microsoft Docs - Connect USB in WSL](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)
- [WSL2 Linux Kernel (GitHub)](https://github.com/microsoft/WSL2-Linux-Kernel/)
- [VideoIO v4l2 /dev/video0 select timeout issue (OpenCV Forum)](https://forum.opencv.org/t/videoio-v4l2-dev-video0-select-timeout/8822)

### Additional Tutorials:
- [Package setup for both Cpp and Python](https://youtu.be/RoRq4XpDEtQ)
- [Launch file for multiple nodes](https://youtu.be/xJ3WAs8GndA)
- [YOLOv8 on ROS2](https://youtu.be/XqibXP4lwgA)
- [Multi-threaded execution](https://youtu.be/amQzXVkR7lY)
- [Yaml parameters](https://youtu.be/wY8MrBGVxYA)
- [Custom msgs](https://youtu.be/E_xBPI8SQig)

### Gazebo Simulation:
- [Gazebo Fortress - Getting Started](https://gazebosim.org/docs/fortress/getstarted/)
- [TurtleBot3 Simulation Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

--- 

## **Contributing**
Contributions are welcome! Please fork the repository and create a pull request.

## **👩‍💻 Author**
Developed by Shivani Khandelwal
For questions or improvements, feel free to reach out at [shivanikhandelwal487@gmail.com](mailto:shivanikhandelwal487@gmail.com).