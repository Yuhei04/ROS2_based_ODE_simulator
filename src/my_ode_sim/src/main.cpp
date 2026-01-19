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
#include "robot.hpp"
#include "lidar.hpp"
#include "world.hpp"

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

static const dReal STEP_SIZE = 0.01;


// --- グローバル変数 ---

// 現在のキー状態
static int g_key = 's';  // 最初は停止

// --- 差動二輪のモータ設定 ---

static void setWheelMotor(dReal v_l, dReal v_r)
{
    const dReal FMAX = 5.0;  // 最大トルク

    dJointSetHingeParam(robot::getLeftJoint(),  dParamVel,  v_l);
    dJointSetHingeParam(robot::getLeftJoint(),  dParamFMax, FMAX);

    dJointSetHingeParam(robot::getRightJoint(), dParamVel,  v_r);
    dJointSetHingeParam(robot::getRightJoint(), dParamFMax, FMAX);
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
    const dReal R = robot::WHEEL_RADIUS;  // 車輪半径
    const dReal T = robot::TRACK;         // トレッド

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

// --- 1ステップごとの処理 ---

static void simLoop(int pause)
{
    if (!pause) {
        controlRobot();

        // LiDAR ビーム更新（ロボットの位置と STEP_SIZE を渡す）
        lidar::update(robot::getChassisBody(), STEP_SIZE);

        // ODE world を 1 ステップ進める（衝突判定 + 積分）
        world::step(STEP_SIZE);

        // オドメトリ更新
        updateOdometry(
            robot::getLeftJoint(),
            robot::getRightJoint(),
            robot::WHEEL_RADIUS,
            robot::TRACK,
            STEP_SIZE);

        // ★ ROS2 のコールバックを処理（/cmd_vel を受け取る）
        if (g_node) {
            rclcpp::spin_some(g_node);   // /cmd_vel を受け取る
            publishOdometryAndTF();      // /odom + /tf
            lidar::publish();
        }
    }

    // --- シャーシの描画 ---
    const dReal* cpos = dBodyGetPosition(robot::getChassisBody());
    const dReal* cR   = dBodyGetRotation(robot::getChassisBody());
    dReal chassisSize[3] = {robot::CHASSIS_L, robot::CHASSIS_W, robot::CHASSIS_H};
    dsDrawBox(cpos, cR, chassisSize);

    // --- 車輪の描画 ---
    const dReal* lwPos = dBodyGetPosition(robot::getLeftWheelBody());
    const dReal* lwR   = dBodyGetRotation(robot::getLeftWheelBody());
    const dReal* rwPos = dBodyGetPosition(robot::getRightWheelBody());
    const dReal* rwR   = dBodyGetRotation(robot::getRightWheelBody());

    dsDrawCylinder(lwPos, lwR, robot::WHEEL_WIDTH, robot::WHEEL_RADIUS);
    dsDrawCylinder(rwPos, rwR, robot::WHEEL_WIDTH, robot::WHEEL_RADIUS);

    // --- キャスタの描画 ---
    if (robot::getCasterBody()) {
        const dReal *cp   = dBodyGetPosition(robot::getCasterBody());
        const dReal *cRot = dBodyGetRotation(robot::getCasterBody());
        dsDrawSphere(cp, cRot, robot::CASTER_RADIUS);
    }

    // --- 壁・障害物の描画 ---
    world::draw();

    // ★ 真値とオドメトリの差を表示
    printOdometryAndError(robot::getChassisBody());
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
    float xyz[3] = { -3.0f, 0.0f, 4.0f };
    float hpr[3] = { 0.0f, -25.0f, 0.0f };
    dsSetViewpoint(xyz, hpr);
}

// --- main ---

int main(int argc, char** argv)
{
    // --- ROS2 初期化 ---
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("ode_diffdrive_sim");

    auto cmd_sub = g_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, cmdVelCallback);

    // --- LiDAR モードを ROS2 パラメータから決める ---
    bool use_3d_lidar = false;
    g_node->declare_parameter<bool>("use_3d_lidar", use_3d_lidar);
    g_node->get_parameter("use_3d_lidar", use_3d_lidar);

    if (use_3d_lidar) {
        lidar::setMode(lidar::Mode::MODE_3D);
        RCLCPP_INFO(g_node->get_logger(), "LiDAR mode: 3D");
    } else {
        lidar::setMode(lidar::Mode::MODE_2D);
        RCLCPP_INFO(g_node->get_logger(), "LiDAR mode: 2D");
    }

    // LiDAR 用 ROS インターフェース初期化（/scan, /points publisher）
    lidar::initRosInterfaces(g_node);

    // /odom, TF は main 側のまま
    g_odom_pub = g_node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    g_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(g_node);

    // --- world type を ROS2 パラメータで決める ---
    std::string world_type;
    g_node->declare_parameter<std::string>("world_type", "corridor");
    g_node->get_parameter("world_type", world_type);

    if (world_type == "simple") {
        RCLCPP_INFO(g_node->get_logger(), "World type: SIMPLE");
        world::init(world::WorldType::SIMPLE);
    } else {
        RCLCPP_INFO(g_node->get_logger(), "World type: CORRIDOR");
        world::init(world::WorldType::CORRIDOR);
    }

    // --- ロボット生成（シャーシ + 車輪 + キャスタ） ---
    robot::create(world::getWorld(), world::getSpace());

    // --- LiDAR Ray の生成 ---
    lidar::initRays(world::getSpace());

    // --- オドメトリ初期化 ---
    initOdometry(robot::getChassisBody());

    // --- drawstuff 設定 ---
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = &command;
    fn.stop    = NULL;
    fn.path_to_textures = "/home/yuhei/ode-0.16.2/drawstuff/textures";

    dsSimulationLoop(argc, argv, 800, 600, &fn);

    // ODE world の後片付け
    world::shutdown();

    return 0;
}