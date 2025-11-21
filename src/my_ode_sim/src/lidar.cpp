#include "my_ode_sim/lidar.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <limits>
#include <cmath>

// ロボットの前向きヨー角を取るために robot のヘッダも使う場合はインクルードしても良いが、
// ここでは chassisBody から直接姿勢を読むので不要。

namespace lidar
{
    // --- 内部用定数（元 main.cpp の LiDAR 部分） ---

    // 2D LiDAR パラメータ
    static const int   LIDAR_BEAMS      = 181;
    static const dReal LIDAR_MAX_RANGE  = 8.0;
    static const dReal LIDAR_FOV        = M_PI * 2.0;
    static const dReal LIDAR_MIN_ANGLE  = -LIDAR_FOV / 2.0;
    static const dReal LIDAR_ANG_STEP   = LIDAR_FOV / (LIDAR_BEAMS - 1);
    static const dReal LIDAR_HEIGHT     = 0.25;

    // 3D LiDAR (mid360 近似)
    static const int   LIDAR3D_H_BEAMS      = 180;
    static const int   LIDAR3D_V_BEAMS      = 16;

    static const dReal LIDAR3D_H_FOV        = 2.0 * M_PI;
    static const dReal LIDAR3D_H_MIN_ANGLE  = -M_PI;
    static const dReal LIDAR3D_H_STEP       =
        LIDAR3D_H_FOV / static_cast<dReal>(LIDAR3D_H_BEAMS);

    static const dReal LIDAR3D_V_MIN_ANGLE  = (-7.0  * M_PI / 180.0);
    static const dReal LIDAR3D_V_MAX_ANGLE  = (52.0  * M_PI / 180.0);
    static const dReal LIDAR3D_V_FOV        =
        (LIDAR3D_V_MAX_ANGLE - LIDAR3D_V_MIN_ANGLE);
    static const dReal LIDAR3D_V_STEP       =
        LIDAR3D_V_FOV / static_cast<dReal>(LIDAR3D_V_BEAMS - 1);

    // --- 内部状態 ---

    // モード（デフォルト 3D）
    static Mode g_mode = Mode::MODE_3D;

    // 2D
    static dGeomID laserBeam[LIDAR_BEAMS];
    static dReal   laserRange[LIDAR_BEAMS];

    // 3D
    static dGeomID laserBeam3D[LIDAR3D_V_BEAMS][LIDAR3D_H_BEAMS];
    static dReal   laserRange3D[LIDAR3D_V_BEAMS][LIDAR3D_H_BEAMS];

    // ROS2 関係
    static std::shared_ptr<rclcpp::Node> g_node;
    static std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>>  g_scan_pub;
    static std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> g_points_pub;

    // 最新のステップ幅（/scan の scan_time に使う）
    static dReal g_last_step_size = 0.01;

    // --- ヘルパー: シャーシのヨー角を計算 ---
    static dReal getHeading(dBodyID chassisBody)
    {
        const dReal* R = dBodyGetRotation(chassisBody);
        return std::atan2(R[4], R[0]);  // x軸の向きから yaw
    }

    // --- 2D レイ更新 ---
    static void updateLidarRays2D(dBodyID chassisBody)
    {
        const dReal* p   = dBodyGetPosition(chassisBody);
        dReal        yaw = getHeading(chassisBody);
        dReal        origin_z = p[2] + LIDAR_HEIGHT;

        for (int i = 0; i < LIDAR_BEAMS; ++i) {
            dReal angle = yaw + (LIDAR_MIN_ANGLE + LIDAR_ANG_STEP * i);
            dReal dx = std::cos(angle);
            dReal dy = std::sin(angle);
            dReal dz = 0.0;

            dGeomRaySet(laserBeam[i],
                        p[0], p[1], origin_z,
                        dx, dy, dz);

            laserRange[i] = LIDAR_MAX_RANGE;
        }
    }

    // --- 3D レイ更新 ---
    static void updateLidarRays3D(dBodyID chassisBody)
    {
        const dReal* p   = dBodyGetPosition(chassisBody);
        dReal        yaw = getHeading(chassisBody);
        dReal        origin_z = p[2] + LIDAR_HEIGHT;

        for (int v = 0; v < LIDAR3D_V_BEAMS; ++v) {
            dReal pitch = LIDAR3D_V_MIN_ANGLE + LIDAR3D_V_STEP * v;

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

    // --- /scan publish ---
    static void publishLaserScan()
    {
        if (!g_node || !g_scan_pub) return;

        auto now = g_node->get_clock()->now();

        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp    = now;
        scan_msg.header.frame_id = "base_link";

        scan_msg.angle_min       = LIDAR_MIN_ANGLE;
        scan_msg.angle_max       = LIDAR_MIN_ANGLE + LIDAR_ANG_STEP * (LIDAR_BEAMS - 1);
        scan_msg.angle_increment = LIDAR_ANG_STEP;

        scan_msg.range_min = 0.05;
        scan_msg.range_max = LIDAR_MAX_RANGE;

        scan_msg.scan_time      = g_last_step_size;
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

    // --- /points publish (3D) ---
    static void publishPointCloud3D()
    {
        if (!g_node || !g_points_pub) return;

        auto now = g_node->get_clock()->now();

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp    = now;
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
                dReal r     = laserRange3D[v][h];

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

    // -------- ここから公開APIの実装 --------

    void setMode(Mode m)
    {
        g_mode = m;
    }

    void initRosInterfaces(const std::shared_ptr<rclcpp::Node>& node)
    {
        g_node = node;

        g_scan_pub = g_node->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", 10);

        g_points_pub = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/points", 10);
    }

    void initRays(dSpaceID space)
    {
        // 2D
        for (int i = 0; i < LIDAR_BEAMS; ++i) {
            laserBeam[i] = dCreateRay(space, LIDAR_MAX_RANGE);
            dGeomRaySetLength(laserBeam[i], LIDAR_MAX_RANGE);
            laserRange[i] = LIDAR_MAX_RANGE;
        }

        // 3D
        for (int v = 0; v < LIDAR3D_V_BEAMS; ++v) {
            for (int h = 0; h < LIDAR3D_H_BEAMS; ++h) {
                laserBeam3D[v][h] = dCreateRay(space, LIDAR_MAX_RANGE);
                dGeomRaySetLength(laserBeam3D[v][h], LIDAR_MAX_RANGE);
                laserRange3D[v][h] = LIDAR_MAX_RANGE;
            }
        }
    }

    void update(dBodyID chassisBody, dReal step_size)
    {
        g_last_step_size = step_size;

        if (g_mode == Mode::MODE_2D) {
            updateLidarRays2D(chassisBody);
        } else {
            updateLidarRays3D(chassisBody);
        }
    }

    // nearCallback から呼ばれる：Ray が何かに当たったときに
    void onRayHit(dGeomID rayGeom, dReal depth)
    {
        if (g_mode == Mode::MODE_2D) {
            for (int k = 0; k < LIDAR_BEAMS; ++k) {
                if (rayGeom == laserBeam[k]) {
                    if (depth < laserRange[k]) {
                        laserRange[k] = depth;
                    }
                    return;
                }
            }
        } else {
            for (int v = 0; v < LIDAR3D_V_BEAMS; ++v) {
                for (int h = 0; h < LIDAR3D_H_BEAMS; ++h) {
                    if (rayGeom == laserBeam3D[v][h]) {
                        if (depth < laserRange3D[v][h]) {
                            laserRange3D[v][h] = depth;
                        }
                        return;
                    }
                }
            }
        }
    }

    void publish()
    {
        if (g_mode == Mode::MODE_2D) {
            publishLaserScan();
        } else {
            publishPointCloud3D();
        }
    }

} // namespace lidar