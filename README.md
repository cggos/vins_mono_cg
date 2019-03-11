# vins_mono_cg

Modified version of [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) (commit 9e657be on Jan 9, 2019)

* **A Robust and Versatile Monocular Visual-Inertial State Estimator**

VINS-Mono uses an optimization-based sliding window formulation for providing high-accuracy visual-inertial odometry. It features efficient IMU pre-integration with bias correction, automatic estimator initialization, online extrinsic calibration, failure detection and recovery, loop detection, and global pose graph optimization, map merge, pose graph reuse, online temporal calibration, rolling shutter support.

-----

# Build

```
catkin_make
```

# Run

```sh
roslaunch vins_estimator euroc.launch
rosbag play YOUR_PATH_TO_DATASET/MH_01_easy.bag
```

* [Ubuntu 16.04 下 VINS-Mono 的安装和使用(RealSense ZR300)](https://blog.csdn.net/u011178262/article/details/88086952)

# Related Papers

```bibtex
@inproceedings{qin2018online,
  title={Online Temporal Calibration for Monocular Visual-Inertial Systems},
  author={Qin, Tong and Shen, Shaojie},
  booktitle={2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={3662--3669},
  year={2018},
  organization={IEEE}
}

@article{qin2017vins,
  title={VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator},
  author={Qin, Tong and Li, Peiliang and Shen, Shaojie},
  journal={IEEE Transactions on Robotics},
  year={2018},
  volume={34},
  number={4},
  pages={1004-1020}
}
```
