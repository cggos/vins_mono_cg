#pragma once

#include "../utility/utility.h"
#include "../parameters.h"

#include <ceres/ceres.h>
using namespace Eigen;

class IntegrationBase
{
  public:
    IntegrationBase() = delete;
    IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg) :
            acc_0{_acc_0},
            gyr_0{_gyr_0},
            linearized_acc{_acc_0},
            linearized_gyr{_gyr_0},
            linearized_ba{_linearized_ba},
            linearized_bg{_linearized_bg},
            jacobian{Eigen::Matrix<double, 15, 15>::Identity()},
            covariance{Eigen::Matrix<double, 15, 15>::Zero()},
            sum_dt{0.0},
            delta_p{Eigen::Vector3d::Zero()},
            delta_q{Eigen::Quaterniond::Identity()},
            delta_v{Eigen::Vector3d::Zero()}

    {
        noise = Eigen::Matrix<double, 18, 18>::Zero();
        noise.block<3, 3>(0, 0)   =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(3, 3)   =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(6, 6)   =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(9, 9)   =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(12, 12) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(15, 15) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
    }

    /**
     * @brief 存入IMU数据，并进行预积分
     * 
     * @param dt  [时间间隔]
     * @param acc [加速度数据]
     * @param gyr [陀螺仪数据]
     */
    void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
    {
        dt_buf.push_back(dt);
        acc_buf.push_back(acc);
        gyr_buf.push_back(gyr);

        //! 进入预积分的阶段
        propagate(dt, acc, gyr);
    }

    /**
     * @brief 
     * 
     * @param _linearized_ba 
     * @param _linearized_bg 
     */
    void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
    {
        sum_dt = 0.0;
        acc_0 = linearized_acc;
        gyr_0 = linearized_gyr;
        delta_p.setZero();
        delta_q.setIdentity();
        delta_v.setZero();
        linearized_ba = _linearized_ba;
        linearized_bg = _linearized_bg;
        jacobian.setIdentity();
        covariance.setZero();
        
        for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
            propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);
    }

    /**
     * @brief 预积分主程序，使用中值法在离散时间下积分
     * 
     * @param _dt                  [时间段]
     * @param _acc_0               [上次加速度值，如果该预计分阶段的首次积分，则上次和本次的两个测量值是一致的]
     * @param _acc_1               [本次加速度值]
     * @param delta_p              [上次的预积分结果，初始值为0]
     * @param linearized_bg        [上次的预积分的陀螺仪bias]
     * @param result_delta_p       [本次的预积分结果]
     * @param result_linearized_ba [本次的预积分的加速度bias]
     * @param update_jacobian      [是否更新雅克比矩阵]
     */
    void midPointIntegration(double _dt, 
                            const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                            const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
    {
        Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
        result_delta_q  = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);

        Vector3d un_acc_0 =        delta_q * (_acc_0 - linearized_ba);
        Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
        Vector3d un_acc   = 0.5 * (un_acc_0 + un_acc_1);

        result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
        result_delta_v = delta_v + un_acc * _dt;

        // 预积分的过程中Bias没有发生改变
        result_linearized_ba = linearized_ba;
        result_linearized_bg = linearized_bg;

        // 离散状态下在计算协方差矩阵的时候为：P' = FPF' + GQG'
        // F矩阵的行顺序为：P, Q, V, Ba, Bg
        if(update_jacobian)
        {
            Matrix3d R_w_x   = Utility::skewSymmetric(0.5 * (_gyr_0 + _gyr_1) - linearized_bg);
            Matrix3d R_a_0_x = Utility::skewSymmetric(_acc_0 - linearized_ba);
            Matrix3d R_a_1_x = Utility::skewSymmetric(_acc_1 - linearized_ba);

            MatrixXd F = MatrixXd::Zero(15, 15);
            F.block<3, 3>(0, 0)   = Matrix3d::Identity();
            F.block<3, 3>(0, 3)   = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
                                    -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
            F.block<3, 3>(0, 6)   = MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(0, 9)   = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
            F.block<3, 3>(0, 12)  = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
            F.block<3, 3>(3, 3)   = Matrix3d::Identity() - R_w_x * _dt;
            F.block<3, 3>(3, 12)  = -1.0 * MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(6, 3)   = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
                                    -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F.block<3, 3>(6, 6)   = Matrix3d::Identity();
            F.block<3, 3>(6, 9)   = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            F.block<3, 3>(6, 12)  = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
            F.block<3, 3>(9, 9)   = Matrix3d::Identity();
            F.block<3, 3>(12, 12) = Matrix3d::Identity();

            MatrixXd V = MatrixXd::Zero(15,18);
            V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * _dt * 0.5 * _dt;
            V.block<3, 3>(0, 6) =  0.25 *  result_delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 9) =  V.block<3, 3>(0, 3);
            V.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(3, 9) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * 0.5 * _dt;
            V.block<3, 3>(6, 6) =  0.5 *  result_delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 9) =  V.block<3, 3>(6, 3);
            V.block<3, 3>(9, 12)  = MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(12, 15) = MatrixXd::Identity(3,3) * _dt;

            jacobian   = F * jacobian;
            covariance = F * covariance * F.transpose() + V * noise * V.transpose();
        }
    }

    /**
     * @brief IMU预积分
     * 
     * @param _dt 时间间隔
     * @param _acc_1 加速度计数据
     * @param _gyr_1 陀螺仪数据
     */
    void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
    {
        dt = _dt;
        acc_1 = _acc_1;
        gyr_1 = _gyr_1;

        Vector3d    result_delta_p;
        Quaterniond result_delta_q;
        Vector3d    result_delta_v;
        Vector3d    result_linearized_ba;
        Vector3d    result_linearized_bg;

        midPointIntegration(
            _dt, 
            acc_0, gyr_0, _acc_1, _gyr_1, 
            delta_p, delta_q, delta_v,
            linearized_ba, linearized_bg,
            result_delta_p, result_delta_q, result_delta_v,
            result_linearized_ba, result_linearized_bg, true);

        // checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v, linearized_ba, linearized_bg);

        delta_p = result_delta_p;
        delta_q = result_delta_q;
        delta_v = result_delta_v;
        linearized_ba = result_linearized_ba;
        linearized_bg = result_linearized_bg;
        delta_q.normalize();

        sum_dt += dt;

        acc_0 = acc_1;
        gyr_0 = gyr_1;
    }

    /**
     * @brief 计算IMU测量模型的残差
     *        Pi，Qi，Vi，Bai，Bgi  [前一次预积分结果]
     *        Pj，Qj，Vj，Baj，Bgj  [后一次预积分结果]
     * @param Pi 
     * @param Qi 
     * @param Vi 
     * @param Bai 
     * @param Bgi 
     * @param Pj 
     * @param Qj 
     * @param Vj 
     * @param Baj 
     * @param Bgj 
     * @return Eigen::Matrix<double, 15, 1> 
     */
    Eigen::Matrix<double, 15, 1> evaluate(
            const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
            const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
    {
        Eigen::Matrix<double, 15, 1> residuals;

        Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

        Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

        Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

        // [cggos] why
        Eigen::Vector3d dba = Bai - linearized_ba;
        Eigen::Vector3d dbg = Bgi - linearized_bg;

        // IMU预积分的结果,消除掉acc bias和gyro bias的影响, 对应IMU model中的\hat{\alpha},\hat{\beta},\hat{\gamma}
        Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
        Eigen::Vector3d    corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
        Eigen::Vector3d    corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

        // IMU项residual计算,输入参数是状态的估计值, 上面correct_delta_*是预积分值, 二者求'diff'得到residual
        residuals.block<3, 1>(O_P, 0)  = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
        residuals.block<3, 1>(O_R, 0)  = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        residuals.block<3, 1>(O_V, 0)  = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
        residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;

        return residuals;
    }

    double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;

    const Eigen::Vector3d linearized_acc, linearized_gyr;
    Eigen::Vector3d linearized_ba, linearized_bg;

    Eigen::Matrix<double, 15, 15> jacobian, covariance;
    Eigen::Matrix<double, 15, 15> step_jacobian;
    Eigen::Matrix<double, 15, 18> step_V;
    Eigen::Matrix<double, 18, 18> noise;

    double sum_dt;
    Eigen::Vector3d    delta_p;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d    delta_v;

    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyr_buf;
};
