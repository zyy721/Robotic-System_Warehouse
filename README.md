# Mobile Order-Fulfillment Robotic System for Warehouse Automation

An archival mobile-manipulation prototype for multi-item order fulfillment. The system integrates order planning, mobile navigation, RGB-D perception, UR5 manipulation, and suction grasping in a real warehouse-style environment.

[![Watch the warehouse robot demo](https://img.youtube.com/vi/gZjYgG2KMVY/hqdefault.jpg)](https://www.youtube.com/watch?v=gZjYgG2KMVY)

**[Watch the end-to-end system demo](https://www.youtube.com/watch?v=gZjYgG2KMVY)**

**Resources:** [Demo video](https://www.youtube.com/watch?v=gZjYgG2KMVY) · [Unpublished technical report](./mobile_order_fulfillment_technical_report_2020.pdf)

## System Pipeline

1. Receive a multi-item order and determine a picking sequence.
2. Navigate the mobile platform between shelves and workstations.
3. Detect target objects from Kinect V2 RGB-D observations.
4. Estimate suction points using image and point-cloud geometry.
5. Position the lift and execute UR5 pick-and-place motions.
6. Transfer the selected items to the order basket.

## System Components

- Mobile robot base
- UR5 manipulator and suction gripper
- Kinect V2 RGB-D camera
- Custom vertical lift
- ROS-based task coordination
- MoveIt motion planning
- OpenCV and PCL perception

## My Contribution

This was a collaborative team project. I contributed to the development and integration of the end-to-end real-robot prototype. The released source is a team-code snapshot and does not imply sole authorship.

## Repository Structure

```text
Robotic-System/
├── include/                 # Perception, device, and robot-control interfaces
├── src/
│   ├── pic_recognize.cpp    # RGB-D and point-cloud processing
│   ├── robot_control.cpp    # UR5 and MoveIt manipulation
│   ├── device_controler.cpp # Mobile base, lift, and suction interfaces
│   ├── decision_maker.cpp   # Picking decisions
│   └── task_receiver.cpp    # ROS task coordination
├── CMakeLists.txt
└── package.xml
```

## Original Software Stack

- Ubuntu 16.04 and ROS Kinetic
- C++11 and catkin
- MoveIt
- OpenCV, PCL, and Kinect V2
- UR5 manipulator with a suction gripper

## Project Status

This repository is an archival snapshot of a 2018–2019 team research prototype and is not actively maintained. The released source is platform-specific and does not include every custom message, detector model, calibration file, hardware interface, or experiment configuration required to reproduce the complete system. It should be treated as an implementation reference rather than a turnkey robotics package.

No open-source license has been specified. Please contact the authors before reusing or redistributing the code.
