#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <cstdlib>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
using namespace Eigen;
using namespace std;

struct SFMFeature {
  bool state;
  int id;
  vector<pair<int, Vector2d>> observation;
  double position[3];
  double depth;
};

struct ReprojectionError3D {
  ReprojectionError3D(double observed_u, double observed_v) : observed_u(observed_u), observed_v(observed_v) {}

  template <typename T>
  bool operator()(const T *const camera_R, const T *const camera_T, const T *point, T *residuals) const {
    T p[3];
    ceres::QuaternionRotatePoint(camera_R, point, p);
    p[0] += camera_T[0];
    p[1] += camera_T[1];
    p[2] += camera_T[2];
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];
    residuals[0] = xp - T(observed_u);
    residuals[1] = yp - T(observed_v);
    return true;
  }

  static ceres::CostFunction *Create(const double observed_x, const double observed_y) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError3D, 2, 4, 3, 3>(
        new ReprojectionError3D(observed_x, observed_y)));
  }

  double observed_u;
  double observed_v;
};

class GlobalSFM {
 public:
  GlobalSFM();

  /**
   * @brief 以第l帧坐标系为参考系，PnP计算滑窗内每一帧的位姿，并三角化特征点，最后 full BA。
   *
   *        输出 q、T： transform from Ci to Cl (第l帧)
   *
   * @param frame_num
   * @param q
   * @param T
   * @param l
   * @param relative_R
   * @param relative_T
   * @param sfm_f
   * @param sfm_tracked_points
   * @return true
   * @return false
   */
  bool construct(int frame_num,
                 Quaterniond *q,
                 Vector3d *T,
                 int l,
                 const Matrix3d relative_R,
                 const Vector3d relative_T,
                 vector<SFMFeature> &sfm_f,
                 map<int, Vector3d> &sfm_tracked_points);

 private:
  bool solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial, int i, vector<SFMFeature> &sfm_f);

  void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0,
                        Eigen::Matrix<double, 3, 4> &Pose1,
                        Vector2d &point0,
                        Vector2d &point1,
                        Vector3d &point_3d);

  /**
   * @brief 在所有特征中 寻找 frame0 和 frame1 有共同观测的特征，并利用观测三角化
   *
   * @param frame0
   * @param Pose0
   * @param frame1
   * @param Pose1
   * @param sfm_f
   */
  void triangulateTwoFrames(int frame0,
                            Eigen::Matrix<double, 3, 4> &Pose0,
                            int frame1,
                            Eigen::Matrix<double, 3, 4> &Pose1,
                            vector<SFMFeature> &sfm_f);

  int feature_num;
};