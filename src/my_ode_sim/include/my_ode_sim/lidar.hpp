#pragma once

#include <memory>
#include <ode/ode.h>
#include <rclcpp/rclcpp.hpp>

namespace lidar
{
    // LiDAR モード（2D or 3D）
    enum class Mode {
        MODE_2D,
        MODE_3D
    };

    // モードの設定（起動時に ROS パラメータから設定）
    void setMode(Mode m);

    // ROS2 ノードを渡して publisher を初期化
    void initRosInterfaces(const std::shared_ptr<rclcpp::Node>& node);

    // ODE の space を渡して Ray Geom を生成
    void initRays(dSpaceID space);

    // 1ステップごとに LiDAR レイを更新（ロボットのボディと dt を渡す）
    void update(dBodyID chassisBody, dReal step_size);

    // nearCallback から呼ぶ：Ray に衝突があったときの処理
    void onRayHit(dGeomID rayGeom, dReal depth);

    // ROS2 に /scan または /points を publish
    void publish();
}