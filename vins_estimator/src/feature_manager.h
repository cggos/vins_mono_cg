#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <algorithm>
#include <list>
#include <numeric>
#include <vector>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/assert.h>
#include <ros/console.h>

#include "parameters.h"

class FeaturePerFrame {
 public:
  FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td) {
    point.x() = _point(0);
    point.y() = _point(1);
    point.z() = _point(2);
    uv.x() = _point(3);
    uv.y() = _point(4);
    velocity.x() = _point(5);
    velocity.y() = _point(6);
    cur_td = td;
  }
  double cur_td;
  Vector3d point;
  Vector2d uv;
  Vector2d velocity;
  double z;
  bool is_used;
  double parallax;
  MatrixXd A;
  VectorXd b;
  double dep_gradient;
};

class FeaturePerId {
 public:
  const int feature_id;
  int start_frame;

  vector<FeaturePerFrame> feature_per_frame;

  int used_num;
  bool is_outlier;
  bool is_margin;
  double estimated_depth;
  int solve_flag;  // 0 haven't solve yet; 1 solve succ; 2 solve fail;

  Vector3d gt_p;

  FeaturePerId(int _feature_id, int _start_frame)
      : feature_id(_feature_id), start_frame(_start_frame), used_num(0), estimated_depth(-1.0), solve_flag(0) {}

  int endFrame();
};

class FeatureManager {
 public:
  FeatureManager(Matrix3d _Rs[]);

  void setRic(Matrix3d _ric[]);

  void clearState();

  int getFeatureCount();

  /**
   * @brief 基于视差来选择关键帧(经过旋转补偿)，向FeaturesManger中添加Features并确定共视关系及视差角的大小
   * @details 需要注意的是，在前段的Feature_tracking部分，对所有的Features都进行了编号
   *          关于视差的判断，并没有看到关于角度的补偿，或者是在后面边缘化的具体过程中存在角度的补偿
   * @param frame_count
   * @param image
   * @param td
   * @return
   */
  bool addFeatureCheckParallax(int frame_count,
                               const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
                               double td);

  vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

  void setDepth(const VectorXd &x);

  void removeFailures();

  void clearDepth(const VectorXd &x);

  /**
   * @brief 读取特征点的逆深度
   * @return
   */
  VectorXd getDepthVector();

  /**
   * @brief 三角化没有恢复出深度的特征点
   * @details 1.将所有的frame转到同一个frame之下  2.进行三角化的剩余步骤
   * @param Ps
   * @param tic
   * @param ric
   */
  void triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);

  void removeBackShiftDepth(Eigen::Matrix3d marg_R,
                            Eigen::Vector3d marg_P,
                            Eigen::Matrix3d new_R,
                            Eigen::Vector3d new_P);
  void removeBack();
  void removeFront(int frame_count);
  void removeOutlier();

  list<FeaturePerId> feature;
  int last_track_num;

 private:
  double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
  const Matrix3d *Rs;
  Matrix3d ric[NUM_OF_CAM];
};

#endif