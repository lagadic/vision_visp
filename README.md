ViSP stack for ROS
==================

![GPL-2](https://www.gnu.org/graphics/gplv3-127x51.png)

## 1. Introduction

ROS 2 vision_visp contains packages to interface ROS 2 with [ViSP](https://visp.inria.fr) which is a library designed 
for visual-servoing and visual tracking applications. This repository contains:

- visp_bridge: Bridge between ROS 2 image and geometry messages and ViSP image and 3D transformation representation.
- visp_tracker: ViSP model-based tracker interfaced in ROS 2 and initialized from a client that requires user interaction.
- visp_auto_tracker: ViSP model-based tracker interfaced in ROS 2 and initialized thanks to a marker (AprilTag, 
  QRcode, flashcode). Recovers when tracking fails.
- visp_camera_calibration: ViSP based tool to calibrate camera intrinsic parameters.
- visp_handeye_calibration: ViSP based tool to estimated the robot end-effector to camera geometric transformation.

##  2. Install dependencies
### 2.1. Install ROS 2

Firstly, it assumes that the ROS 2 core has already been installed, please refer to 
[ROS 2 installation](https://docs.ros.org/en/rolling/Installation.html) to get started.

### 2.2. Install ViSP

Please refer to the official installation guide from [ViSP installation tutorials](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial_install.html).

## 3. Build vision_visp

Fetch the latest code and build

```
$ cd <YOUR_ROS2_WORKSPACE>/src
$ git clone https://github.com/lagadic/vision_visp.git -b rolling
$ cd ..
$ colcon build --symlink-install
```

If ViSP is not found, use `VISP_DIR` to point to `$VISP_WS/visp-build` folder like:

```
$ colcon build --symlink-install --cmake-args -DVISP_DIR=$VISP_WS/visp-build
```

## 4. Usage

- To run `visp_auto_tracker` launch:

  ```
  $ ros2 launch visp_auto_tracker tutorial_launch.xml
  ```

- To run `visp_tracker` launch:

  ```
  $ ros2 launch visp_tracker tutorial_launch.xml
  ```