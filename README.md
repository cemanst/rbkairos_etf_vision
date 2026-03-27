# Edge Vision and Local Control for RB-Kairos

This project is developed within the Robotics Laboratory at the School of Electrical Engineering, University of Belgrade. It implements an optimized edge computer vision pipeline on the NVIDIA Jetson Nano for autonomous robotic manipulation.

The core of this project is to offload computationally intensive vision tasks (fruit detection) from the robot's main computer to an edge device (Jetson Nano). This enables low-latency perception and reliable execution of "detect-and-grasp" actions using the RB-Kairos mobile base and Franka-Emika Panda manipulator.

---

## System Architecture & Technical Specifications

*   **Hardware:** NVIDIA Jetson Nano (Edge Node), Intel RealSense D435 (Camera), RB-Kairos Mobile Base, Franka-Emika Panda (Manipulator).
*   **Software Stack:** ROS Melodic, CUDA 10.2, TensorRT, OpenCV, C++14, Python 3.
*   **Perception:** Custom YOLO model optimized via TensorRT for GPU-accelerated inference, maintaining stable ~20 FPS.

---

## Stack overview

### 1. yolo_node.cpp (The Perception Engine)
This C++ node runs exclusively on the Jetson Nano. It subscribes to the compressed RealSense camera stream. The node uses **TensorRT** and **CUDA** to perform high-speed YOLO inference directly on the GPU. It processes the image, applies Non-Maximum Suppression (NMS), and publishes the bounding boxes as a `vision_msgs::Detection2DArray` on the `/fruit_detections` topic.

### 2. get_base_pose.py (The 3D Tracker)
This script runs on the master PC and bridges the gap between 2D pixels and 3D world coordinates. It synchronizes the 2D detections with the camera's depth map using `message_filters`. It extracts the depth (Z) for the center of the bounding box and uses the camera's intrinsic parameters (K) to calculate the physical X, Y, Z position relative to the camera. It then transforms these coordinates into the robot's base frame (`fr3_link0`).

### 3. detect_and_grasp.py (The State Machine)
This node ensures the robotic arm only moves when the detection is stable. It collects 3D coordinates from the tracker into a buffer of 25 samples. It calculates the standard deviation; if the deviation is below a threshold (< 0.015m), it confirms the object is stationary, calculates the average position, and triggers the grasping action.

### 4. run_grasp_action.py (The Action Caller)
Triggered by the state machine, this script communicates with the `rbkairos_etf_services` via a ROS Service client. It formats the coordinates (adding a 10.3cm offset on the Z-axis to account for the end-effector link) and sends the `grasp_fruit` command. Upon success, it automatically chains the `load_to_bin` action.

### 5. static_transform.py & HandEyeCalibration
To map camera coordinates to robot coordinates, the camera's exact position relative to the arm must be known. `HandEyeCalibration` uses a Charuco board to calculate the transformation matrix. `static_transform.py` then converts this matrix into a quaternion and provides the exact `rosrun tf2_ros static_transform_publisher` command.

---

## System setup & execution


This pipeline depends on the custom action services provided by the laboratory.

*   On your master PC, ensure the services package is installed in `~/catkin_ws/src`.
*   Clone this vision repository into the same workspace:
    `git clone https://github.com/cemanst/rbkairos_edge_vision.git`
*   Build the workspace:
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```


*   Ensure the Jetson Nano is on the same local network as the master PC.
*   Via SSH, copy your ONNX model to the Jetson.
*   Optimize the YOLO model for the Jetson GPU architecture:
    ```bash
    /usr/src/tensorrt/bin/trtexec --onnx=your_model.onnx --saveEngine=yolov5n.engine
    ```
*   Update the engine path in `yolo_node.cpp` and build the package on the Jetson.


*   Start the core components (RealSense, TF, and robot services) on the Master PC:
    ```bash
    roslaunch rbkairos_edge_vision vision_full.launch
    ```
*   On the Jetson Nano, run the perception node:
    ```bash
    rosrun rbkairos_edge_vision yolo_node
    ```


*   Run the visualizer on the master PC to verify detections:
    ```bash
    rosrun rbkairos_edge_vision visualize_detections.py
    ```
*   **Bin Configuration:** If you moved the physical bin, update the coordinates in `run_grasp_action.py`.
*   To get new bin coordinates, move the arm to the bin and run:
    ```bash
    rosrun tf tf_echo fr3_link0 fr3_EE
    ```

 Executing the Grasp
*   Run the main controller:
    ```bash
    rosrun rbkairos_edge_vision detect_and_grasp.py
    ```
*   The system is now active. Place a fruit (apple/banana) on the surface.
*   The system will detect the object, wait for stability, and execute the grasp and drop-off sequence autonomously.
