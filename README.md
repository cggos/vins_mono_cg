# vins_mono_cg

Modified version of [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) (commit 9e657be on Jan 9, 2019)

* **A Robust and Versatile Monocular Visual-Inertial State Estimator**

VINS-Mono uses an optimization-based sliding window formulation for providing high-accuracy visual-inertial odometry. It features efficient IMU pre-integration with bias correction, automatic estimator initialization, online extrinsic calibration, failure detection and recovery, loop detection, and global pose graph optimization, map merge, pose graph reuse, online temporal calibration, rolling shutter support.

[1] *VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator*    
[2] *Online Temporal Calibration for Monocular Visual-Inertial Systems*

-----

# Build

```
catkin_make
```

# Run

* with Dataset

  - MH_01_easy.bag
  ```sh
  roslaunch vins_estimator euroc.launch
  rosbag play YOUR_PATH_TO_DATASET/MH_01_easy.bag
  ```

* with live camera

  - [Ubuntu 16.04 下 VINS-Mono 的安装和使用(RealSense ZR300)](https://blog.csdn.net/u011178262/article/details/88086952)
  ```sh
  roslaunch maplab_realsense maplab_realsense.launch
  roslaunch vins_estimator realsense_fisheye.launch
  ```
  

# Tutorial

* [VINS-Mono代码分析总结](https://www.zybuluo.com/Xiaobuyi/note/866099) by Xiaobuyi
* [VINS-Mono issues 14](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/issues/14): Question about mid-point integration in integration_base.h


# Related Code

* [HKUST-Aerial-Robotics/VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion): An optimization-based multi-sensor state estimator
* [gaowenliang/vins_so](https://github.com/gaowenliang/vins_so): A Robust and Versatile Visual-Inertial State Estimator support Omnidirectional Camera and/or Stereo Camera
* [castiel520/VINS-Mono](https://github.com/castiel520/VINS-Mono): VINS-Mono中文注释
* [QingSimon/VINS-Mono-code-annotation](https://github.com/QingSimon/VINS-Mono-code-annotation): VINS-Mono代码注释以及公式推导
* [heguixiang/Android-VINS](https://github.com/heguixiang/Android-VINS): a version of HKUST-Aerial-Robotics/VINS-Mono running on Android OS
* [pjrambo/VINS-Fusion-gpu](https://github.com/pjrambo/VINS-Fusion-gpu): a version of VINS-Fusion with GPU acceleration
