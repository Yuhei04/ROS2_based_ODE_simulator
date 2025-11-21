// localization.h
#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <ode/ode.h>

struct Pose2D {
    dReal x;
    dReal y;
    dReal theta; // rad
};

// オドメトリ状態を初期化（原点・姿勢0）
void initOdometry(const dBodyID chassisBody);

// 左右車輪のジョイント情報からオドメトリを更新
void updateOdometry(dJointID leftJoint, dJointID rightJoint,
                    dReal wheelRadius, dReal track, dReal dt);

// 現在のオドメトリ姿勢を取得
Pose2D getOdometry();

// 真値とオドメトリの差分を printf で出力
void printOdometryAndError(const dBodyID chassisBody);

#endif