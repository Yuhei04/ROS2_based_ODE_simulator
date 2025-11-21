#pragma once

#include <ode/ode.h>

// ロボット（シャーシ＋左右車輪＋キャスタ）に関する機能をまとめた名前空間
namespace robot
{
    // --- ロボットの基本パラメータ（main.cpp からも使えるように extern 宣言） ---
    extern const dReal CHASSIS_L;   // シャーシ長さ X
    extern const dReal CHASSIS_W;   // 幅 Y
    extern const dReal CHASSIS_H;   // 高さ Z
    extern const dReal CHASSIS_X0;  // 初期位置 X
    extern const dReal CHASSIS_Y0;  // 初期位置 Y;

    extern const dReal WHEEL_RADIUS;
    extern const dReal WHEEL_WIDTH;

    extern const dReal TRACK;          // トレッド
    extern const dReal WHEEL_OFFSET_Y; // ±Y
    extern const dReal WHEEL_OFFSET_X; // 後輪の X オフセット
    extern const dReal CASTER_OFFSET_X;
    extern const dReal CASTER_RADIUS;

    // ロボットを生成（シャーシ＋左右車輪＋キャスタ）
    void create(dWorldID world, dSpaceID space);

    // ボディ・ジョイントへのアクセサ
    dBodyID  getChassisBody();
    dBodyID  getLeftWheelBody();
    dBodyID  getRightWheelBody();
    dBodyID  getCasterBody();

    dJointID getLeftJoint();
    dJointID getRightJoint();

    // LiDAR の Ray 衝突処理用：この Geom がロボットの一部か？
    bool isRobotGeom(dGeomID geom);
}