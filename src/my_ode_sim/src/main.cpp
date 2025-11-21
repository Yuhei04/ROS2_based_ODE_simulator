// v06: 差動二輪 + キャスタ + オドメトリ（ファイル分割版）
//  f: 前進, b: 後退, l: 左旋回, r: 右旋回, s: 停止

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <limits>

#include "localization.h"

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawSphere   dsDrawSphereD
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


// --- グローバル定数（ロボット・シミュレーションパラメータ）---

// ROS2 ノードと /cmd_vel 用のグローバル
std::shared_ptr<rclcpp::Node> g_node;
geometry_msgs::msg::Twist g_cmd_vel;

std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>>      g_odom_pub;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>>  g_scan_pub;
std::shared_ptr<tf2_ros::TransformBroadcaster>                   g_tf_broadcaster;

// /cmd_vel コールバック
void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    g_cmd_vel = *msg;
}

// --- ロボットの基本パラメータ ---
static const dReal CHASSIS_L   = 0.6;   // シャーシ長さ X
static const dReal CHASSIS_W   = 0.4;   // 幅 Y
static const dReal CHASSIS_H   = 0.2;   // 高さ Z
static const dReal CHASSIS_X0  = -2.0;  // シャーシ中心の初期 X
static const dReal CHASSIS_Y0  = -2.0;  // シャーシ中心の初期 Y

static const dReal WHEEL_RADIUS = 0.12;
static const dReal WHEEL_WIDTH  = 0.06;

static const dReal STEP_SIZE = 0.01;

// トレッド（左右タイヤの間隔）
static const dReal TRACK          = 0.5;
static const dReal WHEEL_OFFSET_Y = TRACK / 2.0;

// ★ 後輪の前後位置（シャーシ中心から“後ろ”方向）
static const dReal WHEEL_OFFSET_X = CHASSIS_L / 2.0 - 0.05;
//      → 車体の後端より少し手前くらいにタイヤがくる

// ★ キャスタの前後位置（シャーシ中心から“前”方向）
static const dReal CASTER_OFFSET_X = CHASSIS_L / 2.0 - 0.05;
static const dReal CASTER_RADIUS   = 0.05;

// --- 2D LiDAR パラメータ ---
static const int   LIDAR_BEAMS      = 181;        // 本数（-90〜+90度を1度刻み）
static const dReal LIDAR_MAX_RANGE  = 8.0;        // 最大距離[m]
static const dReal LIDAR_FOV        = M_PI * 2;       // 視野角 360deg
static const dReal LIDAR_MIN_ANGLE  = -LIDAR_FOV / 2.0;            // -90deg
static const dReal LIDAR_ANG_STEP   = LIDAR_FOV / (LIDAR_BEAMS-1); // 角度刻み
static const dReal LIDAR_HEIGHT     = 0.25;       // 地面からの高さ

// --- 3D LiDAR (mid360 近似) パラメータ ---
// mid360: 水平360°, 垂直 -7°〜52° → 59° FOV
// static const int   LIDAR3D_H_BEAMS      = 720; // 水平方向 0.5° 刻み相当
// static const int   LIDAR3D_V_BEAMS      = 40;  // 「40 line 相当」に近い

static const int   LIDAR3D_H_BEAMS      = 180; // 水平方向 0.5° 刻み相当
static const int   LIDAR3D_V_BEAMS      = 16;  // 「40 line 相当」に近い

static const dReal LIDAR3D_H_FOV        = 2.0 * M_PI;
static const dReal LIDAR3D_H_MIN_ANGLE  = -M_PI;
static const dReal LIDAR3D_H_STEP       =
    LIDAR3D_H_FOV / static_cast<dReal>(LIDAR3D_H_BEAMS);

static const dReal LIDAR3D_V_MIN_ANGLE  = (-7.0  * M_PI / 180.0);  // -7°
static const dReal LIDAR3D_V_MAX_ANGLE  = (52.0  * M_PI / 180.0);  // 52°
static const dReal LIDAR3D_V_FOV        = (LIDAR3D_V_MAX_ANGLE - LIDAR3D_V_MIN_ANGLE);
static const dReal LIDAR3D_V_STEP       =
    LIDAR3D_V_FOV / static_cast<dReal>(LIDAR3D_V_BEAMS - 1);

// 3D 用ビーム配列
static dGeomID laserBeam3D[LIDAR3D_V_BEAMS][LIDAR3D_H_BEAMS];
static dReal   laserRange3D[LIDAR3D_V_BEAMS][LIDAR3D_H_BEAMS];

// --- グローバル変数 ---

static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;

// シャーシ（本体）
static dBodyID chassisBody;
static dGeomID chassisGeom;

// 車輪（左・右）
static dBodyID leftWheelBody, rightWheelBody;
static dGeomID leftWheelGeom, rightWheelGeom;
static dJointID leftJoint, rightJoint;

// キャスタ
static dBodyID casterBody;
static dGeomID casterGeom;
static dJointID casterJoint;

// --- LiDAR 用グローバル ---
static dGeomID laserBeam[LIDAR_BEAMS];
static dReal   laserRange[LIDAR_BEAMS];
static bool    g_showLaser = true;

// LiDAR モード
enum class LidarMode {
    MODE_2D,
    MODE_3D
};

// デフォルトは 3D
LidarMode g_lidar_mode = LidarMode::MODE_3D;

std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> g_points_pub;

// 壁＆障害物
static dGeomID wall1, wall2, wall3, wall4;
static dGeomID obs1, obs2;

// 現在のキー状態
static int g_key = 's';  // 最初は停止

// --- 衝突コールバック ---

static void nearCallback(void* data, dGeomID o1, dGeomID o2)
{
    const int N = 10;
    dContact contact[N];

    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    int class1 = dGeomGetClass(o1);
    int class2 = dGeomGetClass(o2);

    // --- ① LiDAR Ray の場合の特別処理 ---
    // --- ★★ Ray の衝突処理（2D + 3D 両対応）★★ ---
    if (class1 == dRayClass || class2 == dRayClass) {
        dGeomID rayGeom = (class1 == dRayClass) ? o1 : o2;
        dGeomID objGeom = (rayGeom == o1) ? o2 : o1;

        // 自分のボディは無視
        if (objGeom == chassisGeom ||
            objGeom == leftWheelGeom || objGeom == rightWheelGeom ||
            objGeom == casterGeom) {
            return;
        }

        int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
        if (n <= 0) return;

        dReal minDepth = LIDAR_MAX_RANGE;
        for (int i = 0; i < n; ++i) {
            if (contact[i].geom.depth < minDepth) {
                minDepth = contact[i].geom.depth;
            }
        }

        // --- LiDAR モードごとに、見る配列を変える ---
        if (g_lidar_mode == LidarMode::MODE_2D) {
            // 2D ビームだけ探す
            for (int k = 0; k < LIDAR_BEAMS; ++k) {
                if (rayGeom == laserBeam[k]) {
                    if (minDepth < laserRange[k]) {
                        laserRange[k] = minDepth;
                    }
                    return;
                }
            }
        } else { // MODE_3D
            // 3D ビームだけ探す
            for (int v = 0; v < LIDAR3D_V_BEAMS; ++v) {
                for (int h = 0; h < LIDAR3D_H_BEAMS; ++h) {
                    if (rayGeom == laserBeam3D[v][h]) {
                        if (minDepth < laserRange3D[v][h]) {
                            laserRange3D[v][h] = minDepth;
                        }
                        return;
                    }
                }
            }
        }

        return;
    }

    // --- ② それ以外（通常の剛体同士の接触） ---
    // ロボット内部同士（ジョイントでつながってるもの）は除外
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;

    int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
    for (int i = 0; i < n; ++i) {
        contact[i].surface.mode =
            dContactSlip1 | dContactSlip2 |
            dContactSoftERP | dContactSoftCFM | dContactApprox1;

        contact[i].surface.mu  = 1.0;
        contact[i].surface.slip1    = 0.001;
        contact[i].surface.slip2    = 0.001;
        contact[i].surface.soft_erp = 1.0;
        contact[i].surface.soft_cfm = 1e-5;

        dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
        dJointAttach(c,
            dGeomGetBody(contact[i].geom.g1),
            dGeomGetBody(contact[i].geom.g2));
    }
}

// --- 差動二輪のモータ設定 ---

static void setWheelMotor(dReal v_l, dReal v_r)
{
    const dReal FMAX = 5.0;  // 最大トルク

    dJointSetHingeParam(leftJoint,  dParamVel,  v_l);
    dJointSetHingeParam(leftJoint,  dParamFMax, FMAX);

    dJointSetHingeParam(rightJoint, dParamVel,  v_r);
    dJointSetHingeParam(rightJoint, dParamFMax, FMAX);
}

// --- キー状態に応じてモータ目標速度を決める ---
// （方向関係は v04 で調整した通りに）

static void controlRobot()
{
    // /cmd_vel から並進・回転速度を取得
    const dReal v_cmd = g_cmd_vel.linear.x;   // [m/s]
    const dReal w_cmd = g_cmd_vel.angular.z;  // [rad/s]

    // ★ ここで符号を反転して、昔の f/b, l/r の挙動に合わせる
    const dReal v = -v_cmd;   // i で前進（v_cmd>0）→ 内部では v<0 → ちゃんと前に進む
    const dReal w = -w_cmd;   // j で左旋回（w_cmd>0）→ 内部では w<0 → ちゃんと左に回る

    // 差動二輪モデル
    const dReal R = WHEEL_RADIUS;  // 車輪半径
    const dReal T = TRACK;         // トレッド

    // ωR, ωL [rad/s]
    const dReal omega_r = (2.0 * v + w * T) / (2.0 * R);
    const dReal omega_l = (2.0 * v - w * T) / (2.0 * R);

    setWheelMotor(omega_l, omega_r);
}

// /odom と /tf を publish する関数

static void publishOdometryAndTF()
{
    if (!g_node || !g_odom_pub || !g_tf_broadcaster) return;

    Pose2D odo = getOdometry();  // localization.h のやつ

    auto now = g_node->get_clock()->now();

    // --- Odometry メッセージ ---
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = "odom";       // 親座標
    odom_msg.child_frame_id = "base_link";   // ロボット本体

    odom_msg.pose.pose.position.x = odo.x;
    odom_msg.pose.pose.position.y = odo.y;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odo.theta);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // 速度はとりあえず 0 でOK（後で余裕があれば v, w から埋めてもよい）
    odom_msg.twist.twist.linear.x  = g_cmd_vel.linear.x;
    odom_msg.twist.twist.angular.z = g_cmd_vel.angular.z;

    g_odom_pub->publish(odom_msg);

    // --- TF (odom -> base_link) ---
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";

    tf_msg.transform.translation.x = odo.x;
    tf_msg.transform.translation.y = odo.y;
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

    g_tf_broadcaster->sendTransform(tf_msg);
}

// ロボットの前向きのヨー角を取得（すでにあれば重複不要）
static dReal getHeading()
{
    const dReal *R = dBodyGetRotation(chassisBody);
    return std::atan2(R[4], R[0]);  // x軸の向きから yaw を計算
}

// LiDAR ビームの位置・向きを更新
static void updateLidarRays()
{
    const dReal *p = dBodyGetPosition(chassisBody);
    dReal yaw = getHeading();

    dReal origin_z = p[2] + LIDAR_HEIGHT;

    for (int i = 0; i < LIDAR_BEAMS; ++i) {
        dReal angle = yaw + (LIDAR_MIN_ANGLE + LIDAR_ANG_STEP * i);
        dReal dx = std::cos(angle);
        dReal dy = std::sin(angle);
        dReal dz = 0.0;

        dGeomRaySet(laserBeam[i],
                    p[0], p[1], origin_z,   // 始点
                    dx, dy, dz);            // 向き

        // このステップの初期値として「見えてない＝最大距離」
        laserRange[i] = LIDAR_MAX_RANGE;
    }
}

static void updateLidarRays3D()
{
    const dReal *p = dBodyGetPosition(chassisBody);
    dReal yaw = getHeading();
    dReal origin_z = p[2] + LIDAR_HEIGHT;

    for (int v = 0; v < LIDAR3D_V_BEAMS; ++v) {
        dReal pitch = LIDAR3D_V_MIN_ANGLE + LIDAR3D_V_STEP * v;  // 垂直角

        for (int h = 0; h < LIDAR3D_H_BEAMS; ++h) {

            dReal yaw_h = yaw + (LIDAR3D_H_MIN_ANGLE + LIDAR3D_H_STEP * h);

            dReal dx = std::cos(pitch) * std::cos(yaw_h);
            dReal dy = std::cos(pitch) * std::sin(yaw_h);
            dReal dz = std::sin(pitch);

            dGeomRaySet(laserBeam3D[v][h],
                        p[0], p[1], origin_z,
                        dx, dy, dz);

            laserRange3D[v][h] = LIDAR_MAX_RANGE;
        }
    }
}

// /scan を publish する関数


static void publishLaserScan()
{
    if (!g_node || !g_scan_pub) return;

    auto now = g_node->get_clock()->now();

    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.stamp = now;
    scan_msg.header.frame_id = "base_link";

    scan_msg.angle_min       = LIDAR_MIN_ANGLE;
    scan_msg.angle_max       = LIDAR_MIN_ANGLE + LIDAR_ANG_STEP * (LIDAR_BEAMS - 1);
    scan_msg.angle_increment = LIDAR_ANG_STEP;

    scan_msg.range_min = 0.05;              // 0 より少し大きく
    scan_msg.range_max = LIDAR_MAX_RANGE;

    scan_msg.scan_time = STEP_SIZE;
    scan_msg.time_increment = 0.0;

    scan_msg.ranges.resize(LIDAR_BEAMS);
    scan_msg.intensities.resize(LIDAR_BEAMS);

    for (int i = 0; i < LIDAR_BEAMS; ++i) {
        float r = static_cast<float>(laserRange[i]);

        if (r <= scan_msg.range_min || r >= scan_msg.range_max) {
            scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
        } else {
            scan_msg.ranges[i] = r;
        }

        scan_msg.intensities[i] = 1.0f;
    }

    g_scan_pub->publish(scan_msg);
}

static void publishPointCloud3D()
{
    if (!g_node || !g_points_pub) return;

    auto now = g_node->get_clock()->now();

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now;
    cloud.header.frame_id = "base_link";

    const int POINTS = LIDAR3D_V_BEAMS * LIDAR3D_H_BEAMS;
    cloud.height = 1;
    cloud.width  = POINTS;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(POINTS);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    int idx = 0;
    for (int v = 0; v < LIDAR3D_V_BEAMS; ++v) {
        dReal pitch = LIDAR3D_V_MIN_ANGLE + LIDAR3D_V_STEP * v;

        for (int h = 0; h < LIDAR3D_H_BEAMS; ++h, ++idx, ++iter_x, ++iter_y, ++iter_z) {

            dReal yaw_h = LIDAR3D_H_MIN_ANGLE + LIDAR3D_H_STEP * h;
            dReal r = laserRange3D[v][h];

            if (r >= LIDAR_MAX_RANGE || r <= 0.0) {
                *iter_x = std::numeric_limits<float>::quiet_NaN();
                *iter_y = std::numeric_limits<float>::quiet_NaN();
                *iter_z = std::numeric_limits<float>::quiet_NaN();
                continue;
            }

            dReal dx = std::cos(pitch) * std::cos(yaw_h);
            dReal dy = std::cos(pitch) * std::sin(yaw_h);
            dReal dz = std::sin(pitch);

            *iter_x = static_cast<float>(dx * r);
            *iter_y = static_cast<float>(dy * r);
            *iter_z = static_cast<float>(dz * r);
        }
    }

    g_points_pub->publish(cloud);
}

// LiDAR レイの更新（モードごとの分岐）
// 今は 2D/3D どちらも 2D の updateLidarRays() を呼ぶ（枠だけ作る）
static void updateLidar()
{
    if (g_lidar_mode == LidarMode::MODE_2D) {
        updateLidarRays();          // 既存の 2D LiDAR 用関数
    } else {
        // TODO: 3D LiDAR 実装時に 3D 用の updateLidarRays3D() に切り替える
        updateLidarRays3D();          // 今は 2D と同じ動きをさせておく
    }
}

// LiDAR データ publish（モードごとの分岐）
static void publishLidar()
{
    if (g_lidar_mode == LidarMode::MODE_2D) {
        publishLaserScan();         // 既存の /scan publish 関数
    } else {
        // TODO: 3D 実装時に PointCloud2 publish などに切り替える
        publishPointCloud3D();         // 今は /scan をそのまま出す（2Dと同じ）
    }
}

// --- 1ステップごとの処理 ---

static void simLoop(int pause)
{
    if (!pause) {
        controlRobot();

        // LiDAR ビーム更新＋衝突判定
        updateLidar();
        dSpaceCollide(space, 0, &nearCallback);
        dWorldStep(world, STEP_SIZE);
        dJointGroupEmpty(contactgroup);

        // ★ オドメトリ更新
        updateOdometry(leftJoint, rightJoint,
                       WHEEL_RADIUS, TRACK, STEP_SIZE);

        // ★ ROS2 のコールバックを処理（/cmd_vel を受け取る）
        if (g_node) {
            rclcpp::spin_some(g_node);   // /cmd_vel を受け取る
            publishOdometryAndTF();      // /odom + /tf
            publishLidar();
        }
    }

    // --- シャーシの描画 ---
    const dReal* cpos = dBodyGetPosition(chassisBody);
    const dReal* cR   = dBodyGetRotation(chassisBody);
    dReal chassisSize[3] = {CHASSIS_L, CHASSIS_W, CHASSIS_H};
    dsDrawBox(cpos, cR, chassisSize);

    // --- 車輪の描画 ---
    const dReal* lwPos = dBodyGetPosition(leftWheelBody);
    const dReal* lwR   = dBodyGetRotation(leftWheelBody);
    const dReal* rwPos = dBodyGetPosition(rightWheelBody);
    const dReal* rwR   = dBodyGetRotation(rightWheelBody);

    dsDrawCylinder(lwPos, lwR, WHEEL_WIDTH, WHEEL_RADIUS);
    dsDrawCylinder(rwPos, rwR, WHEEL_WIDTH, WHEEL_RADIUS);

    // --- キャスタの描画 ---
    if (casterBody) {
        const dReal *cp = dBodyGetPosition(casterBody);
        const dReal *cRot = dBodyGetRotation(casterBody);
        dsDrawSphere(cp, cRot, CASTER_RADIUS);
    }

    // --- 壁 ---
    if (wall1) {
        dReal s1[3] = {0.2, 10.0, 2.0};
        dsDrawBox(dGeomGetPosition(wall1), dGeomGetRotation(wall1), s1);
    }
    if (wall2) {
        dReal s2[3] = {0.2, 10.0, 2.0};
        dsDrawBox(dGeomGetPosition(wall2), dGeomGetRotation(wall2), s2);
    }
    if (wall3) {
        dReal s3[3] = {10.0, 0.2, 2.0};
        dsDrawBox(dGeomGetPosition(wall3), dGeomGetRotation(wall3), s3);
    }
    if (wall4) {
        dReal s4[3] = {10.0, 0.2, 2.0};
        dsDrawBox(dGeomGetPosition(wall4), dGeomGetRotation(wall4), s4);
    }

    // --- 障害物 ---
    if (obs1) {
        dReal o1[3] = {1.0, 1.0, 1.5};
        dsDrawBox(dGeomGetPosition(obs1), dGeomGetRotation(obs1), o1);
    }
    if (obs2) {
        dReal o2[3] = {1.5, 0.3, 1.0};
        dsDrawBox(dGeomGetPosition(obs2), dGeomGetRotation(obs2), o2);
    }

    // ★ 真値とオドメトリの差を表示
    printOdometryAndError(chassisBody);
}

// --- キー入力 ---
//  f:前, b:後, l:左, r:右, s:停止

static void command(int cmd)
{
    switch (cmd) {
        case 'f':
        case 'b':
        case 'l':
        case 'r':
        case 's':
            g_key = cmd;
            break;
        default:
            break;
    }
}

// --- カメラ初期設定 ---

static void start()
{
    float xyz[3] = { 6.0f, -6.0f, 4.0f };
    float hpr[3] = { 135.0f, -30.0f, 0.0f };
    dsSetViewpoint(xyz, hpr);
}

// --- main ---

int main(int argc, char** argv)
{
    // --- ROS2 初期化 ---
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("ode_diffdrive_sim");

    // /cmd_vel サブスクライバを作成
    auto cmd_sub = g_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, cmdVelCallback);

    g_points_pub = g_node->create_publisher<sensor_msgs::msg::PointCloud2>("/points", 10);

    // --- LiDAR モードを ROS2 パラメータから決める ---
    // use_3d_lidar = true なら 3D モード / false なら 2D モード
    bool use_3d_lidar = true;  // ★ デフォルト true = 3D

    g_node->declare_parameter<bool>("use_3d_lidar", use_3d_lidar);
    g_node->get_parameter("use_3d_lidar", use_3d_lidar);

    if (use_3d_lidar) {
        g_lidar_mode = LidarMode::MODE_3D;
        RCLCPP_INFO(g_node->get_logger(), "LiDAR mode: 3D");
    } else {
        g_lidar_mode = LidarMode::MODE_2D;
        RCLCPP_INFO(g_node->get_logger(), "LiDAR mode: 2D");
    }

    // /odom, /scan publisher, TF broadcaster を作る
    g_odom_pub = g_node->create_publisher<nav_msgs::msg::Odometry>(
        "/odom", 10);

    g_scan_pub = g_node->create_publisher<sensor_msgs::msg::LaserScan>(
        "/scan", 10);

    g_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(g_node);

    // --- ODE 初期化 ---
    dInitODE();

    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);

    dWorldSetGravity(world, 0, 0, -9.81);

    // 地面
    dCreatePlane(space, 0, 0, 1, 0);

    // --- 壁 ---
    wall1 = dCreateBox(space, 0.2, 10.0, 2.0);
    dGeomSetPosition(wall1, -5.0, 0.0, 1.0);

    wall2 = dCreateBox(space, 0.2, 10.0, 2.0);
    dGeomSetPosition(wall2,  5.0, 0.0, 1.0);

    wall3 = dCreateBox(space, 10.0, 0.2, 2.0);
    dGeomSetPosition(wall3, 0.0, -5.0, 1.0);

    wall4 = dCreateBox(space, 10.0, 0.2, 2.0);
    dGeomSetPosition(wall4, 0.0,  5.0, 1.0);

    // --- 障害物 ---
    obs1 = dCreateBox(space, 1.0, 1.0, 1.5);
    dGeomSetPosition(obs1, 0.0, 0.0, 0.75);

    obs2 = dCreateBox(space, 1.5, 0.3, 1.0);
    dGeomSetPosition(obs2, 2.0, 1.0, 0.5);

    // --- シャーシ ---
    chassisBody = dBodyCreate(world);
    dMass m;
    dMassSetBox(&m, 1.0, CHASSIS_L, CHASSIS_W, CHASSIS_H);
    dMassAdjust(&m, 5.0);
    dBodySetMass(chassisBody, &m);
    dBodySetPosition(chassisBody,
                     CHASSIS_X0,
                     CHASSIS_Y0,
                     CHASSIS_H / 2.0 + 0.02);

    chassisGeom = dCreateBox(space, CHASSIS_L, CHASSIS_W, CHASSIS_H);
    dGeomSetBody(chassisGeom, chassisBody);

    // --- 車輪 ---
    dMass mw;
    dMassSetCylinder(&mw, 1.0, 1, WHEEL_RADIUS, WHEEL_WIDTH);
    dMassAdjust(&mw, 1.0);

    // 左後輪
    leftWheelBody = dBodyCreate(world);
    dBodySetMass(leftWheelBody, &mw);
    dBodySetPosition(leftWheelBody,
        CHASSIS_X0 - WHEEL_OFFSET_X,   // ★ シャーシ中心より“後ろ”
        CHASSIS_Y0 + WHEEL_OFFSET_Y,   // 左(+Y)
        WHEEL_RADIUS);

    leftWheelGeom = dCreateCylinder(space, WHEEL_RADIUS, WHEEL_WIDTH);
    dGeomSetBody(leftWheelGeom, leftWheelBody);

    // 右後輪
    rightWheelBody = dBodyCreate(world);
    dBodySetMass(rightWheelBody, &mw);
    dBodySetPosition(rightWheelBody,
        CHASSIS_X0 - WHEEL_OFFSET_X,   // ★ 同じく“後ろ”
        CHASSIS_Y0 - WHEEL_OFFSET_Y,   // 右(-Y)
        WHEEL_RADIUS);

    rightWheelGeom = dCreateCylinder(space, WHEEL_RADIUS, WHEEL_WIDTH);
    dGeomSetBody(rightWheelGeom, rightWheelBody);

    // タイヤの向きをそろえる（X軸まわり90°回転）
    {
        dMatrix3 R;
        dRFromAxisAndAngle(R, 1, 0, 0, M_PI / 2.0);
        dBodySetRotation(leftWheelBody, R);
        dBodySetRotation(rightWheelBody, R);
    }

    // --- シャーシと車輪のジョイント ---
    leftJoint = dJointCreateHinge(world, 0);
    dJointAttach(leftJoint, chassisBody, leftWheelBody);
    dJointSetHingeAnchor(leftJoint,
        CHASSIS_X0 - WHEEL_OFFSET_X,   // ★ 後ろ
        CHASSIS_Y0 + WHEEL_OFFSET_Y,
        WHEEL_RADIUS);
    dJointSetHingeAxis(leftJoint, 0, 1, 0);

    rightJoint = dJointCreateHinge(world, 0);
    dJointAttach(rightJoint, chassisBody, rightWheelBody);
    dJointSetHingeAnchor(rightJoint,
        CHASSIS_X0 - WHEEL_OFFSET_X,   // ★ 後ろ
        CHASSIS_Y0 - WHEEL_OFFSET_Y,
        WHEEL_RADIUS);
    dJointSetHingeAxis(rightJoint, 0, 1, 0);

    // --- キャスタ（前方） ---
    {
        dMass mc;
        casterBody = dBodyCreate(world);
        dMassSetSphere(&mc, 1.0, CASTER_RADIUS);
        dMassAdjust(&mc, 0.5);
        dBodySetMass(casterBody, &mc);

        // シャーシ中心から“前”方向に CASTER_OFFSET_X だけ離す
        dBodySetPosition(
            casterBody,
            CHASSIS_X0 + CASTER_OFFSET_X,   // ★ 前
            CHASSIS_Y0,
            CASTER_RADIUS);

        casterGeom = dCreateSphere(space, CASTER_RADIUS);
        dGeomSetBody(casterGeom, casterBody);

        casterJoint = dJointCreateBall(world, 0);
        dJointAttach(casterJoint, chassisBody, casterBody);
        dJointSetBallAnchor(
            casterJoint,
            CHASSIS_X0 + CASTER_OFFSET_X,   // ★ 前
            CHASSIS_Y0,
            CASTER_RADIUS);
    }

    // --- 2D LiDAR ビーム生成 ---
    for (int i = 0; i < LIDAR_BEAMS; ++i) {
        laserBeam[i] = dCreateRay(space, LIDAR_MAX_RANGE);
        dGeomRaySetLength(laserBeam[i], LIDAR_MAX_RANGE);
        laserRange[i] = LIDAR_MAX_RANGE;
    }

    // --- 3D LiDAR ビーム生成 ---
    for (int v = 0; v < LIDAR3D_V_BEAMS; ++v) {
        for (int h = 0; h < LIDAR3D_H_BEAMS; ++h) {
            laserBeam3D[v][h] = dCreateRay(space, LIDAR_MAX_RANGE);
            dGeomRaySetLength(laserBeam3D[v][h], LIDAR_MAX_RANGE);
            laserRange3D[v][h] = LIDAR_MAX_RANGE;
        }
    }

    // --- オドメトリ初期化 ---
    initOdometry(chassisBody);

    // --- drawstuff 設定 ---
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = &command;
    fn.stop    = NULL;
    fn.path_to_textures = "/home/ubuntu/ode-0.16.1/drawstuff/textures";

    dsSimulationLoop(argc, argv, 800, 600, &fn);

    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}