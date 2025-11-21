// localization.cpp
#include "localization.h"
#include <cmath>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 内部用：オドメトリ姿勢
static Pose2D s_odo = {0.0, 0.0, 0.0};

// 内部用：回転行列からヨー角（heading）を計算
static dReal headingFromBody(const dBodyID body)
{
    const dReal *R = dBodyGetRotation(body);
    // x軸の向きから平面角を求める
    // R[0] = x軸のx成分, R[4] = x軸のy成分
    return std::atan2(R[4], R[0]);  // [-pi, pi]
}

void initOdometry(const dBodyID chassisBody)
{
    // 物理世界上どこに置いてあっても、
    // オドメトリ座標系の原点を (0,0,0) とする
    s_odo.x     = 0.0;
    s_odo.y     = 0.0;
    s_odo.theta = 0.0;
}

void updateOdometry(dJointID leftJoint, dJointID rightJoint,
                    dReal wheelRadius, dReal track, dReal dt)
{
    // 左右車輪の角速度 [rad/s]
    dReal omega_l = dJointGetHingeAngleRate(leftJoint);
    dReal omega_r = dJointGetHingeAngleRate(rightJoint);

    // 差動二輪モデル
    dReal v = -wheelRadius * (omega_r + omega_l) * 0.5;       // 並進速度 [m/s]
    // dReal w = -wheelRadius * (omega_r - omega_l) / track;     // 角速度 [rad/s]

    // 例：物理パラメータとは別に、オドメトリ用の補正値を使う
    const dReal ODOM_TRACK = 0.55;  // 0.5 → 0.55 とか少しずつ変えてみる
    dReal w = -wheelRadius * (omega_r - omega_l) / ODOM_TRACK;

    // 角度更新
    s_odo.theta += w * dt;

    // 角度正規化
    if (s_odo.theta >  M_PI) s_odo.theta -= 2.0 * M_PI;
    if (s_odo.theta < -M_PI) s_odo.theta += 2.0 * M_PI;

    // 位置更新（世界座標系）
    s_odo.x += v * std::cos(s_odo.theta) * dt;
    s_odo.y += v * std::sin(s_odo.theta) * dt;
}

Pose2D getOdometry()
{
    return s_odo;
}

void printOdometryAndError(const dBodyID chassisBody)
{
    const dReal *p = dBodyGetPosition(chassisBody);
    dReal real_x = p[0];
    dReal real_y = p[1];
    dReal real_theta = headingFromBody(chassisBody);

    Pose2D odo = s_odo;

    dReal real_deg = real_theta * 180.0 / M_PI;
    dReal odo_deg  = odo.theta * 180.0 / M_PI;
    dReal err_theta_deg = (real_theta - odo.theta) * 180.0 / M_PI;

    std::printf("Real:      x,y,theta = (%7.3f, %7.3f, %7.3f deg)\n",
                real_x, real_y, real_deg);
    std::printf("Odometry:  x,y,theta = (%7.3f, %7.3f, %7.3f deg)\n",
                odo.x,  odo.y,  odo_deg);
    std::printf("Error:     dx,dy,dth = (%7.3f, %7.3f, %7.3f deg)\n\n",
                real_x - odo.x,
                real_y - odo.y,
                err_theta_deg);
}